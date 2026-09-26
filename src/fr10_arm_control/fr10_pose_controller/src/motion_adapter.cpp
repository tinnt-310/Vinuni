#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "fr10_pose_controller/cartesian_path.hpp"
#include "fr10_pose_controller/cartesian_route_planner.hpp"
#include "fr10_pose_controller/cartesian_trajectory.hpp"
#include "fr10_pose_controller/collision_validated_ik_provider.hpp"
#include "fr10_pose_controller/continuous_ik.hpp"
#include "fr10_pose_controller/diagnostic_display.hpp"
#include "fr10_pose_controller/fr10_analytical_ik.hpp"
#include "fr10_pose_controller/linear_insert_planner.hpp"
#include "fr10_pose_controller/math_utils.hpp"
#include "fr10_pose_controller/motion_controller.hpp"
#include "fr10_pose_controller/motion_adapter_runtime.hpp"
#include "fr10_pose_controller/moveit_ik_provider.hpp"
#include "fr10_pose_controller/node_parameters.hpp"
#include "fr10_pose_controller/pre_insert_planner.hpp"
#include "fr10_pose_controller/realtime_buffer.hpp"
#include "fr10_pose_controller/robot_backend.hpp"
#include "fr10_pose_controller/ros_control_backend.hpp"

namespace {
using namespace std::chrono_literals;

volatile std::sig_atomic_t stop_signal = 0;
int stop_pipe_write_fd = -1;

void signalHandler(int signal_number) {
  stop_signal = signal_number;
  if (stop_pipe_write_fd >= 0) {
    const uint8_t signal_byte = static_cast<uint8_t>(signal_number);
    const ssize_t write_result =
        ::write(stop_pipe_write_fd, &signal_byte, sizeof(signal_byte));
    (void)write_result;
  }
}

std::size_t policyIndex(fr10_motion::PoseExecutionPolicy policy) {
  return static_cast<std::size_t>(policy);
}

enum class StopReason : int {
  NONE = 0,
  USER_INTERRUPT,
  GOAL_REACHED,
  SINGULARITY,
  HARDWARE_ALARM,
  COMM_FAILURE,
  IK_FAILURE,
  PATH_DEVIATION,
  REFERENCE_LAG,
  SCENE_CHANGED,
  CONTROLLED_STOP,
  RETARGET_REVERSAL
};

const char *stopReasonName(StopReason reason) {
  switch (reason) {
  case StopReason::NONE:
    return "NONE";
  case StopReason::USER_INTERRUPT:
    return "USER_INTERRUPT";
  case StopReason::GOAL_REACHED:
    return "GOAL_REACHED";
  case StopReason::SINGULARITY:
    return "SINGULARITY";
  case StopReason::HARDWARE_ALARM:
    return "HARDWARE_ALARM";
  case StopReason::COMM_FAILURE:
    return "COMM_FAILURE";
  case StopReason::IK_FAILURE:
    return "IK_FAILURE";
  case StopReason::PATH_DEVIATION:
    return "PATH_DEVIATION";
  case StopReason::REFERENCE_LAG:
    return "REFERENCE_LAG";
  case StopReason::SCENE_CHANGED:
    return "SCENE_CHANGED";
  case StopReason::CONTROLLED_STOP:
    return "CONTROLLED_STOP";
  case StopReason::RETARGET_REVERSAL:
    return "RETARGET_REVERSAL";
  }
  return "UNKNOWN";
}

class MotionAdapter final : public rclcpp::Node {
  struct KinematicsTask {
    fr10_motion::ControlMode control_mode{fr10_motion::ControlMode::JOINT};
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget pose_target;
    fr10_motion::RobotState seed;
    fr10_motion::PoseExecutionPolicy policy{
        fr10_motion::PoseExecutionPolicy::FREE_SPACE};
    uint64_t sequence{0};
    unsigned replan_attempt{0};
  };

  struct KinematicsResult {
    fr10_motion::ControlMode control_mode{fr10_motion::ControlMode::JOINT};
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget pose_target;
    fr10_motion::PoseExecutionPolicy policy{
        fr10_motion::PoseExecutionPolicy::FREE_SPACE};
    uint64_t sequence{0};
    unsigned replan_attempt{0};
    uint64_t scene_revision{0};
    double manipulability{0.0};
    double condition_number{std::numeric_limits<double>::infinity()};
    std::size_t ik_waypoint_count{0};
    std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory>
        pose_trajectory;
    double trajectory_duration_s{0.0};
    double planned_max_position_error_m{0.0};
    double planned_max_orientation_error_rad{0.0};
    double planned_max_path_deviation_m{0.0};
    bool used_kinematic_detour{false};
    bool used_alternate_orientation{false};
    bool used_joint_space_detour{false};
    double detour_clearance_m{0.0};
    fr10_motion::PathTimeDiagnostics timing_diagnostics;
    bool requires_controlled_stop_replan{false};
  };

  struct PendingInsertionSequence {
    fr10_motion::CartesianTarget insertion_target;
    fr10_motion::Pose pre_insert_pose;
    uint64_t staging_generation{0};
  };

  struct LoopMetrics {
    uint64_t samples{0};
    double elapsed_s{0.0};
    double mean_abs_jitter_ms{0.0};
    double max_abs_jitter_ms{0.0};
    double max_cycle_period_ms{0.0};
    double max_delta_joint_velocity_rad_s{0.0};
    double max_delta_linear_velocity_m_s{0.0};
    uint64_t command_write_errors{0};
  };

  struct DiagnosticsSnapshot {
    fr10_motion::MotionStatus status;
    fr10_motion::RobotState feedback;
    fr10_motion::MotionCommand command;
    LoopMetrics loop;
  };

  struct StatusLogSnapshot {
    fr10_motion::MotionStatus status;
    fr10_motion::RobotState feedback;
    fr10_motion::MotionCommand command;
    fr10_motion::Pose actual_flange;
    fr10_motion::Pose final_pose_goal;
    LoopMetrics loop;
    bool actual_flange_valid{false};
    bool final_pose_goal_valid{false};
  };

  struct AcceptedTargetSnapshot {
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget cartesian_target;
    bool have_joint_target{false};
    bool have_cartesian_target{false};
  };

  struct DisplayPoseGoalSnapshot {
    fr10_motion::Pose pose;
    bool valid{false};
  };

public:
  explicit MotionAdapter(const rclcpp::NodeOptions &options)
      : Node("fr10_pose_controller", options),
        params_(fr10_pose_controller::NodeParameters::declare(this)) {
    path_fault_monitor_.setConfig(params_.cartesian_fault_monitor_configs[
        policyIndex(fr10_motion::PoseExecutionPolicy::FREE_SPACE)]);
    motion_executor_ =
        std::make_unique<fr10_motion::MotionExecutor>(params_.controller_config);

    control_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    target_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    path_monitor_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    feedback_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Parameters are startup configuration; never accept changes that the
    // live executor would silently ignore.
    parameter_guard_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters) {
            if (parameter.get_name().rfind("controller.", 0) == 0) {
              result.successful = false;
              result.reason = "controller parameters require restart";
              break;
            }
          }
          return result;
        });

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/controller_status", rclcpp::QoS(1).reliable().transient_local());
    tracking_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
        "/fr10/diagnostics/tracking",
        rclcpp::QoS(1).reliable().transient_local());
    tcp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/fr10_motion/tcp_pose", rclcpp::SensorDataQoS());
    rclcpp::SubscriptionOptions target_subscription_options;
    target_subscription_options.callback_group = target_callback_group_;
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        params_.joint_input_topic, rclcpp::QoS(10).reliable(),
        std::bind(&MotionAdapter::jointTargetCallback, this,
                  std::placeholders::_1),
        target_subscription_options);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        params_.pose_input_topic, rclcpp::QoS(1).reliable(),
        std::bind(&MotionAdapter::poseTargetCallback, this,
                  std::placeholders::_1),
        target_subscription_options);
    retract_sub_ = create_subscription<std_msgs::msg::Empty>(
        params_.retract_topic, rclcpp::QoS(1).reliable(),
        std::bind(&MotionAdapter::retractCallback, this, std::placeholders::_1),
        target_subscription_options);
    rclcpp::SubscriptionOptions stop_options;
    stop_options.callback_group = stop_callback_group_;
    controlled_stop_sub_ = create_subscription<std_msgs::msg::Empty>(
        params_.controlled_stop_topic, rclcpp::QoS(1).reliable(),
        std::bind(&MotionAdapter::controlledStopCallback, this,
                  std::placeholders::_1),
        stop_options);

    tcp_pose_message_.header.frame_id = params_.base_frame;
    status_message_.data.reserve(1536);
  }

  ~MotionAdapter() override {
    requestStopAll();
    shutdownBackend();
  }

  void initialize() {
    initializeKinematics();

    if (params_.backend_name != "ros_control") {
      throw std::runtime_error(
          "Pose/Joint runtime requires controller.backend=ros_control");
    }
    backend_ = std::make_unique<fr10_motion::RosControlBackend>(
        shared_from_this(), params_.joint_names, params_.command_topic, params_.joint_state_topic,
        feedback_callback_group_, params_.feedback_timeout_s);
    if (!backend_->initialize()) {
      throw std::runtime_error(std::string("Failed to initialize ") +
                               backend_->name());
    }
    fr10_motion::RobotState initial_state;
    if (backend_->readState(initial_state)) {
      fr10_motion::Pose initial_flange;
      if (!populateCartesianFeedback(initial_state, initial_flange)) {
        throw std::runtime_error(
            "Failed to compute initial FLANGE/TCP feedback");
      }
      feedback_buffer_.write(initial_state);
      fr10_motion::Pose initial_tcp;
      initial_tcp.position_m = initial_state.tcp_position_m;
      initial_tcp.orientation_quat = initial_state.tcp_orientation_quat;
      tcp_feedback_buffer_.write(initial_tcp);
      motion_executor_->update(initial_state, params_.nominal_period_s);
      publishTcpPose(initial_state);
    }

    last_tick_ = std::chrono::steady_clock::now();
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(params_.nominal_period_s)),
        std::bind(&MotionAdapter::controlTick, this), control_callback_group_);
    path_monitor_last_tick_ = std::chrono::steady_clock::now();
    path_monitor_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::duration<double>(params_.nominal_period_s)),
                          std::bind(&MotionAdapter::pathMonitorTick, this),
                          path_monitor_callback_group_);
    diagnostics_timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&MotionAdapter::diagnosticsTick, this),
                          diagnostics_callback_group_);
    status_log_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / params_.diagnostic_log_rate_hz)),
        std::bind(&MotionAdapter::statusLogTick, this),
        diagnostics_callback_group_);
    kinematics_worker_ =
        std::thread(&MotionAdapter::kinematicsWorkerLoop, this);

    RCLCPP_INFO(get_logger(), "[INITIALIZING] Motion core ready; waiting for "
                              "active ros2_control command consumer.");
    DiagnosticsSnapshot initial_snapshot;
    initial_snapshot.status = motion_executor_->status();
    initial_snapshot.feedback = initial_state;
    initial_snapshot.command = motion_executor_->command();
    diagnostics_buffer_.write(initial_snapshot);
    diagnosticsTick();
    statusLogTick();
  }

  void requestStopAll(StopReason reason = StopReason::USER_INTERRUPT) {
    bool expected = false;
    if (!stop_requested_.compare_exchange_strong(expected, true)) {
      return;
    }
    stop_reason_.store(reason);
    if (control_timer_) {
      control_timer_->cancel();
    }
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    if (path_monitor_timer_) {
      path_monitor_timer_->cancel();
    }
    if (status_log_timer_) {
      status_log_timer_->cancel();
    }
    kinematics_cv_.notify_all();
    if (motion_executor_) {
      motion_executor_->stop();
    }
    RCLCPP_INFO(get_logger(), "[SHUTDOWN: %s] 125 Hz control timers stopped.",
                stopReasonName(reason));
  }

  void cleanupWorkers() {
    kinematics_cv_.notify_all();
    if (kinematics_worker_.joinable()) {
      kinematics_worker_.join();
    }
  }

  bool stopRequested() const { return stop_requested_.load(); }

  void beginGracefulShutdown() {
    shutdown_in_progress_.store(true);
    if (!motion_executor_ || !motion_executor_->initialized()) {
      // There is no accepted feedback and therefore no command trajectory to
      // brake. Waiting for HOLDING here can only time out.
      requestStopAll(StopReason::USER_INTERRUPT);
      return;
    }
    stop_latch_.store(StopReason::USER_INTERRUPT);
  }

  double shutdownTimeoutS() const {
    double duration = 0.0;
    const auto &limits = params_.controller_config.limits;
    for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
      duration = std::max(duration,
          limits.max_joint_velocity_rad_s[i] / limits.max_joint_acceleration_rad_s2[i] +
          2.0 * limits.max_joint_acceleration_rad_s2[i] / limits.max_joint_jerk_rad_s3[i]);
    }
    return duration + 0.5;
  }

  bool shutdownBackend() {
    if (backend_shutdown_.exchange(true) || !backend_) {
      return true;
    }
    const std::string backend_name = backend_->name();
    const bool clean = backend_->shutdown();
    backend_.reset();
    RCLCPP_INFO(
        get_logger(),
        "[SHUTDOWN: %s] %s shutdown %s; this does not confirm physical standstill.",
        stopReasonName(stop_reason_.load()), backend_name.c_str(),
        clean ? "completed" : "reported an error");
    return clean;
  }

private:
  void requestControlledStop(StopReason reason) {
    // Invalidate worker output before starting the braking profile so a stale
    // result cannot replace the stop trajectory.
    kinematics_task_sequence_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      pending_kinematics_task_.reset();
    }
    if (reason != StopReason::RETARGET_REVERSAL) {
      std::lock_guard<std::mutex> lock(reversal_mutex_);
      post_stop_replan_task_.reset();
    }
    if (reason != StopReason::RETARGET_REVERSAL) {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      pending_insertion_sequence_.reset();
      last_insertion_target_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      have_queued_cartesian_target_ = false;
    }
    stop_reason_.store(reason);
    if (motion_executor_) {
      motion_executor_->stop();
    }
    pending_event_.store(reason);
  }

  void controlledStopCallback(
      const std_msgs::msg::Empty::ConstSharedPtr & /*message*/) {
    if (stop_requested_.load() || !motion_executor_ ||
        !motion_executor_->initialized()) {
      return;
    }
    RCLCPP_WARN(get_logger(),
                "[CONTROLLED STOP REQUEST] explicit software stop requested");
    stop_latch_.store(StopReason::CONTROLLED_STOP);
  }

  void initializeKinematics() {
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description", true);
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("Failed to load MoveIt robot model");
    }
    joint_model_group_ = robot_model_->getJointModelGroup(params_.kinematics_group);
    if (!joint_model_group_ ||
        joint_model_group_->getVariableCount() != fr10_motion::kJointCount) {
      throw std::runtime_error("Invalid six-axis MoveIt group: " +
                               params_.kinematics_group);
    }
    if (!robot_model_->hasLinkModel(params_.kinematics_tip_link) ||
        !robot_model_->hasLinkModel(params_.kinematics_safety_link) ||
        !robot_model_->hasLinkModel(params_.diagnostic_flange_frame)) {
      throw std::runtime_error(
          "MoveIt model is missing configured tip/safety/diagnostic link");
    }
    const auto &model_joint_names =
        joint_model_group_->getActiveJointModelNames();
    if (model_joint_names != params_.joint_names) {
      throw std::runtime_error(
          "MoveIt group joint order must match controller.joint_names");
    }

    if (params_.collision_checking_enabled) {
      planning_scene_monitor_ =
          std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
              shared_from_this(), robot_model_loader_,
              "fr10_pose_controller_scene_monitor");
      if (!planning_scene_monitor_->getPlanningScene()) {
        throw std::runtime_error(
            "Failed to initialize MoveIt PlanningScene collision monitor");
      }
      planning_scene_monitor_->startSceneMonitor(params_.planning_scene_topic);
      planning_scene_monitor_->startWorldGeometryMonitor(
          params_.collision_object_topic, params_.planning_scene_world_topic, false);
      planning_scene_monitor_->startStateMonitor(params_.joint_state_topic);
      planning_scene_monitor_->addUpdateCallback(
          [this](planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType
                     update_type) {
            const int update = static_cast<int>(update_type);
            const int collision_relevant =
                static_cast<int>(planning_scene_monitor::PlanningSceneMonitor::
                                     UPDATE_TRANSFORMS) |
                static_cast<int>(planning_scene_monitor::PlanningSceneMonitor::
                                     UPDATE_GEOMETRY);
            if ((update & collision_relevant) != 0) {
              scene_revision_.fetch_add(1, std::memory_order_acq_rel);
            }
          });
      RCLCPP_INFO(get_logger(),
                  "PlanningScene collision validation enabled: scene=%s "
                  "objects=%s world=%s",
                  params_.planning_scene_topic.c_str(),
                  params_.collision_object_topic.c_str(),
                  params_.planning_scene_world_topic.c_str());
    }

    fr10_pose_controller::MoveItIKProviderConfig provider_config;
    provider_config.group_name = params_.kinematics_group;
    provider_config.tip_link = params_.kinematics_tip_link;
    provider_config.safety_link = params_.kinematics_safety_link;
    provider_config.timeout_s = params_.ik_timeout_s;
    provider_config.wrist_min_abs_rad = params_.wrist_singularity_min_rad;
    provider_config.elbow_min_abs_rad = params_.elbow_singularity_min_rad;
    provider_config.max_wrist_reach_m = params_.max_wrist_reach_m;
    provider_config.min_wrist_radius_m = params_.min_wrist_radius_m;
    provider_config.max_jacobian_condition = params_.max_jacobian_condition;
    provider_config.collision_checking_enabled = params_.collision_checking_enabled;
    provider_config.planning_scene_monitor = planning_scene_monitor_;
    provider_config.limits = params_.controller_config.limits;
    moveit_ik_provider_ =
        std::make_unique<fr10_pose_controller::MoveItIKProvider>(
            robot_model_, std::move(provider_config));
    path_ik_provider_ = moveit_ik_provider_.get();
    initializeDiagnosticFrames();

    if (params_.analytical_ik_enabled) {
      analytical_ik_provider_ =
          std::make_unique<fr10_motion::Fr10AnalyticalIKProvider>(
              params_.analytical_ik_config);
      const std::array<std::array<double, 6>, 3> validation_states{
          {{{0.9281, -1.9512, 1.9526, -1.6002, -1.2993, 0.0}},
           {{-0.75, -1.35, 1.10, -1.20, 1.05, 0.35}},
           {{0.40, -1.10, -1.25, -1.00, -1.15, -0.40}}}};
      for (const auto &positions : validation_states) {
        fr10_motion::KinematicState state;
        state.position_rad = positions;
        fr10_motion::Pose moveit_pose;
        fr10_motion::Pose analytical_pose;
        if (!moveit_ik_provider_->forward(state, moveit_pose) ||
            !analytical_ik_provider_->forward(state, analytical_pose)) {
          throw std::runtime_error(
              "Analytical/MoveIt FK startup cross-check failed");
        }
        double squared_error = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double delta =
              moveit_pose.position_m[axis] - analytical_pose.position_m[axis];
          squared_error += delta * delta;
        }
        const double position_error = std::sqrt(squared_error);
        const double orientation_error =
            fr10_motion::math::quaternionAngularDistance(
                moveit_pose.orientation_quat, analytical_pose.orientation_quat);
        if (position_error > params_.analytical_ik_model_position_tolerance_m ||
            orientation_error >
                params_.analytical_ik_model_orientation_tolerance_rad) {
          throw std::runtime_error(
              "Analytical IK geometry does not match the loaded MoveIt model");
        }
      }
      collision_validated_ik_provider_ =
          std::make_unique<fr10_pose_controller::CollisionValidatedIKProvider>(
              *analytical_ik_provider_, *moveit_ik_provider_);
      path_ik_provider_ = collision_validated_ik_provider_.get();
      RCLCPP_INFO(get_logger(),
                  "Closed-form FR10 analytical IK enabled and cross-checked "
                  "against MoveIt FK");
    }
  }

  void initializeDiagnosticFrames() {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.update();
    const Eigen::Isometry3d flange_to_tcp =
        state.getGlobalLinkTransform(params_.diagnostic_flange_frame).inverse() *
        state.getGlobalLinkTransform(params_.kinematics_tip_link);
    if (!flange_to_tcp.matrix().allFinite()) {
      throw std::runtime_error(
          "Diagnostic FLANGE-to-TCP transform is non-finite");
    }
    const Eigen::Vector3d translation = flange_to_tcp.translation();
    const Eigen::Quaterniond orientation(flange_to_tcp.rotation());
    flange_to_tcp_.position_m = {translation.x(), translation.y(),
                                 translation.z()};
    flange_to_tcp_.orientation_quat = {orientation.x(), orientation.y(),
                                       orientation.z(), orientation.w()};
    if (!fr10_motion::math::normalizeQuaternion(
            flange_to_tcp_.orientation_quat)) {
      throw std::runtime_error(
          "Diagnostic FLANGE-to-TCP orientation is invalid");
    }

    const std::array<double, 3> specified_translation{0.060, 0.0, 0.175};
    double translation_mismatch_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          flange_to_tcp_.position_m[axis] - specified_translation[axis];
      translation_mismatch_squared += delta * delta;
    }
    const double translation_mismatch_m =
        std::sqrt(translation_mismatch_squared);
    const double rotation_from_identity_rad =
        fr10_motion::math::quaternionAngularDistance(
            flange_to_tcp_.orientation_quat, {0.0, 0.0, 0.0, 1.0});
    const auto rpy = fr10_motion::math::quaternionToFixedXyzRpy(
        flange_to_tcp_.orientation_quat);

    RCLCPP_INFO(get_logger(),
                "[TOOL MODEL] FLANGE=%s TCP=%s model_translation=[%.6f, "
                "%.6f, %.6f] m model_fixed_XYZ_RPY=[%.6f, %.6f, %.6f] deg "
                "source=robot_description/URDF",
                params_.diagnostic_flange_frame.c_str(), params_.kinematics_tip_link.c_str(),
                flange_to_tcp_.position_m[0], flange_to_tcp_.position_m[1],
                flange_to_tcp_.position_m[2], rpy[0] * fr10_motion::kRadToDeg,
                rpy[1] * fr10_motion::kRadToDeg,
                rpy[2] * fr10_motion::kRadToDeg);
    if (translation_mismatch_m > 1e-6) {
      RCLCPP_WARN(get_logger(),
                  "[TOOL MODEL MISMATCH] specified cylinder L=0.150 m D=0.060 "
                  "m R=0.030 m center_flange=[0.060, 0.000, 0.100] m "
                  "tcp_flange=[0.060, 0.000, 0.175] m; current %s->%s "
                  "translation differs by %.3f mm. Model was not modified.",
                  params_.diagnostic_flange_frame.c_str(),
                  params_.kinematics_tip_link.c_str(),
                  translation_mismatch_m * 1000.0);
    } else {
      RCLCPP_INFO(get_logger(), "[TOOL MODEL VERIFIED] specified and modeled "
                                "FLANGE-to-TCP translations match");
    }
    RCLCPP_INFO(
        get_logger(),
        "[TOOL ROTATION VERIFIED] fixed rotation source=robot_description/URDF "
        "identity=%s angular_distance_from_identity=%.9f deg",
        rotation_from_identity_rad <= 1e-9 ? "true" : "false",
        rotation_from_identity_rad * fr10_motion::kRadToDeg);
  }

  bool populateCartesianFeedback(fr10_motion::RobotState &feedback,
                                 fr10_motion::Pose &actual_flange) const {
    if (!moveit_ik_provider_) {
      return false;
    }
    fr10_motion::KinematicState state;
    state.position_rad = feedback.joint_position_rad;
    state.velocity_rad_s = feedback.joint_velocity_rad_s;
    state.acceleration_rad_s2 = feedback.joint_acceleration_rad_s2;
    if (!moveit_ik_provider_->forwardLink(state, params_.diagnostic_flange_frame,
                                          actual_flange)) {
      return false;
    }
    const fr10_motion::Pose actual_tcp =
        fr10_motion::diagnostics::composePose(actual_flange, flange_to_tcp_);
    feedback.tcp_position_m = actual_tcp.position_m;
    feedback.tcp_orientation_quat = actual_tcp.orientation_quat;
    return true;
  }

  bool updateJointIndexMap(const sensor_msgs::msg::JointState &message) {
    if (joint_index_map_valid_ && message.name == joint_command_layout_) {
      return true;
    }

    std::array<bool, fr10_motion::kJointCount> used{};
    std::array<std::size_t, fr10_motion::kJointCount> next_map{};
    for (std::size_t expected = 0; expected < fr10_motion::kJointCount;
         ++expected) {
      bool found = false;
      for (std::size_t source = 0; source < message.name.size(); ++source) {
        if (!used[source] && message.name[source] == params_.joint_names[expected]) {
          next_map[expected] = source;
          used[source] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
    }
    joint_command_index_map_ = next_map;
    joint_command_layout_ = message.name;
    joint_index_map_valid_ = true;
    return true;
  }

  bool feedbackSnapshot(fr10_motion::RobotState &snapshot) const {
    if (!feedback_buffer_.read(snapshot) || !snapshot.valid) { return false; }
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - snapshot.received_steady_s;
    if (age > params_.feedback_timeout_s || age < 0.0) { return false; }
    fr10_motion::KinematicState joints;
    joints.position_rad = snapshot.joint_position_rad;
    fr10_motion::Pose tcp;
    if (!analytical_ik_provider_ || !analytical_ik_provider_->forward(joints, tcp)) { return false; }
    snapshot.tcp_position_m = tcp.position_m;
    snapshot.tcp_orientation_quat = tcp.orientation_quat;
    return true;
  }

  void queueKinematicsTask(const KinematicsTask &task) {
    if (shutdown_in_progress_.load() || stop_latch_.load() != StopReason::NONE ||
        motion_executor_->status().state == fr10_motion::MotionState::STOPPING) { return; }
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      pending_kinematics_task_ = task;
    }
    kinematics_cv_.notify_one();
  }

  void clearRejectedStagingGeneration(uint64_t generation) {
    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      if (pending_insertion_sequence_ &&
          pending_insertion_sequence_->staging_generation == generation) {
        pending_insertion_sequence_.reset();
      }
    }
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (have_queued_cartesian_target_ &&
          last_queued_cartesian_target_.generation == generation) {
        have_queued_cartesian_target_ = false;
      }
    }
  }

  void logTimeParameterDiagnostics(
      uint64_t generation,
      const fr10_motion::PathTimeParameterizationResult &result) const {
    const auto &diagnostics = result.diagnostics;
    RCLCPP_INFO(
        get_logger(),
        "[TIME_PARAM_DIAGNOSTIC] generation=%lu policy=%s\n"
        "Trajectory Distance     : %.3f m | Rotation: %.1f deg | Duration: "
        "%.3f s\n"
        "Max Joint Vel Ratio     : %.3f (Joint %zu)\n"
        "Max Joint Acc Ratio     : %.3f (Joint %zu)\n"
        "Max Joint Jerk Ratio    : %.3f (Joint %zu)\n"
        "Max Linear Vel Ratio    : %.3f | Acc: %.3f | Jerk: %.3f\n"
        "Max Angular Vel Ratio   : %.3f | Acc: %.3f | Jerk: %.3f\n"
        "INITIAL PATH VEL CAP    : %s\n"
        "LIMITING CONSTRAINT     : %s | scaling iterations: %zu",
        generation,
        fr10_motion::poseExecutionPolicyName(diagnostics.policy),
        diagnostics.trajectory_distance_m,
        diagnostics.trajectory_rotation_rad * fr10_motion::kRadToDeg,
        diagnostics.duration_s, diagnostics.max_joint_velocity_ratio,
        diagnostics.max_joint_velocity_index + 1,
        diagnostics.max_joint_acceleration_ratio,
        diagnostics.max_joint_acceleration_index + 1,
        diagnostics.max_joint_jerk_ratio, diagnostics.max_joint_jerk_index + 1,
        diagnostics.linear_velocity_ratio,
        diagnostics.linear_acceleration_ratio, diagnostics.linear_jerk_ratio,
        diagnostics.angular_velocity_ratio,
        diagnostics.angular_acceleration_ratio, diagnostics.angular_jerk_ratio,
        fr10_motion::timeParameterConstraintName(
            diagnostics.initial_velocity_constraint),
        fr10_motion::timeParameterConstraintName(
            diagnostics.limiting_constraint),
        diagnostics.scaling_iterations);
  }

  bool publishKinematicsResult(const KinematicsResult &result) {
    retained_trajectories_.erase(std::remove_if(retained_trajectories_.begin(),
        retained_trajectories_.end(), [](const auto &trajectory) {
          return trajectory.use_count() == 1 && trajectory->trajectoryUseCount() == 1;
        }), retained_trajectories_.end());
    if (result.pose_trajectory) { retained_trajectories_.push_back(result.pose_trajectory); }
    while (!stop_requested_.load() && result.sequence == kinematics_task_sequence_.load()) {
      if (kinematics_result_buffer_.write(result)) { return true; }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  void kinematicsWorkerLoop() {
    while (!stop_requested_.load()) {
      KinematicsTask task;
      {
        std::unique_lock<std::mutex> lock(kinematics_mutex_);
        kinematics_cv_.wait(lock, [this]() {
          return stop_requested_.load() || pending_kinematics_task_.has_value();
        });
        if (stop_requested_.load()) {
          return;
        }
        task = *pending_kinematics_task_;
        pending_kinematics_task_.reset();
      }

      KinematicsResult result;
      result.control_mode = task.control_mode;
      result.sequence = task.sequence;
      result.replan_attempt = task.replan_attempt;
      result.policy = task.policy;
      result.pose_target = task.pose_target;
      result.joint_target = task.joint_target;
      planning_scene::PlanningSceneConstPtr planning_scene_snapshot;
      result.scene_revision = scene_revision_.load(std::memory_order_acquire);
      if (params_.collision_checking_enabled && planning_scene_monitor_) {
        bool captured = false;
        for (unsigned attempt = 0; attempt < 3 && !captured; ++attempt) {
          const uint64_t revision_before =
              scene_revision_.load(std::memory_order_acquire);
          {
            planning_scene_monitor::LockedPlanningSceneRO scene(
                planning_scene_monitor_);
            if (!scene) {
              break;
            }
            planning_scene_snapshot = planning_scene::PlanningScene::clone(
                static_cast<const planning_scene::PlanningSceneConstPtr &>(
                    scene));
          }
          const uint64_t revision_after =
              scene_revision_.load(std::memory_order_acquire);
          captured = planning_scene_snapshot &&
                     revision_before == revision_after;
          if (captured) {
            result.scene_revision = revision_after;
          }
        }
        if (!captured) {
          stop_reason_.store(StopReason::IK_FAILURE);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: SCENE_UNAVAILABLE] sequence=%lu",
                       task.sequence);
          continue;
        }
      }
      moveit_ik_provider_->setPlanningSceneSnapshot(planning_scene_snapshot);
      struct SceneSnapshotReset {
        fr10_pose_controller::MoveItIKProvider *provider;
        ~SceneSnapshotReset() { provider->setPlanningSceneSnapshot({}); }
      } scene_snapshot_reset{moveit_ik_provider_.get()};
      fr10_motion::KinematicState seed;
      seed.position_rad = task.seed.joint_position_rad;
      seed.velocity_rad_s = task.seed.joint_velocity_rad_s;
      seed.acceleration_rad_s2 = task.seed.joint_acceleration_rad_s2;

      if (task.control_mode == fr10_motion::ControlMode::POSE) {
        fr10_motion::Pose start_pose;
        std::string rejection_reason;
        if (!moveit_ik_provider_->forward(seed, start_pose,
                                          &rejection_reason)) {
          clearRejectedStagingGeneration(task.pose_target.generation);
          stop_reason_.store(StopReason::IK_FAILURE);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: IK_FAILURE] FK seed failed: %s",
                       rejection_reason.c_str());
          continue;
        }
        fr10_motion::CartesianTarget effective_target = task.pose_target;
        if (params_.lock_orientation) {
          effective_target.orientation_quat = start_pose.orientation_quat;
        }
        result.pose_target = effective_target;
        const fr10_motion::Pose goal_pose = effective_target.toPose();
        auto route_config = params_.cartesian_route_config;
        route_config.mode =
            task.policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE
                ? fr10_motion::CartesianRouteMode::FREE_SPACE_JOINT
                : fr10_motion::CartesianRouteMode::CARTESIAN_LINEAR;
        if (task.policy == fr10_motion::PoseExecutionPolicy::INSERTION) {
          // Precision mating/unmating never bends around a singular segment.
          route_config.singularity_avoidance_enabled = false;
        }
        auto route = fr10_motion::CartesianRoutePlanner::plan(
            start_pose, goal_pose, seed, *path_ik_provider_, route_config,
            [this, sequence = task.sequence]() {
              return stop_requested_.load() ||
                     sequence != kinematics_task_sequence_.load();
            });
        if (!route.success) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          stop_reason_.store(StopReason::IK_FAILURE);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: IK_FAILURE] generation=%lu "
                       "direct_waypoint=%zu detour_attempts=%zu reason=%s",
                       task.pose_target.generation, route.direct_failure_index,
                       route.detour_attempts, route.message.c_str());
          continue;
        }
        if (route.used_detour) {
          RCLCPP_WARN(
              get_logger(),
              "[KINEMATIC DETOUR] generation=%lu kind=%s direct_waypoint=%zu "
              "direct_reason=%s attempts=%zu clearance=%.3f m direction=[%.3f, "
              "%.3f, %.3f]; PlanningScene validation is applied",
              task.pose_target.generation,
              route.used_alternate_orientation
                  ? "ALTERNATE_ORIENTATION"
                  : (route.used_joint_space_detour ? "JOINT_SPACE_DETOUR"
                                                   : "POSITIONAL_DETOUR"),
              route.direct_failure_index, route.direct_failure_message.c_str(),
              route.detour_attempts, route.detour_clearance_m,
              route.detour_direction[0], route.detour_direction[1],
              route.detour_direction[2]);
        }
        auto &cartesian_path = route.cartesian_path;
        auto &continuous = route.continuous_ik;
        result.used_kinematic_detour = route.used_detour;
        result.used_alternate_orientation = route.used_alternate_orientation;
        result.used_joint_space_detour = route.used_joint_space_detour;
        result.detour_clearance_m = route.detour_clearance_m;
        result.manipulability = continuous.min_manipulability;
        result.condition_number = continuous.max_condition_number;
        result.ik_waypoint_count = continuous.joint_path.size();
        const auto &timing_config =
            params_.path_time_policies[policyIndex(task.policy)];
        auto parameterized = fr10_motion::PathTimeParameterizer::parameterize(
            cartesian_path, continuous.joint_path, seed, timing_config,
            task.pose_target.generation, [this, sequence = task.sequence]() {
              return stop_requested_.load() ||
                     sequence != kinematics_task_sequence_.load();
            });
        if (parameterized.success || parameterized.duration_s > 0.0) {
          logTimeParameterDiagnostics(task.pose_target.generation,
                                      parameterized);
        }
        if (!parameterized.success) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          if (parameterized.status == fr10_motion::TimeParameterizationStatus::
                                          REQUIRES_CONTROLLED_STOP) {
            result.requires_controlled_stop_replan = true;
            result.timing_diagnostics = parameterized.diagnostics;
            if (!publishKinematicsResult(result)) {
              RCLCPP_WARN(get_logger(),
                          "Retarget-stop mailbox busy; dropping sequence=%lu",
                          result.sequence);
            }
            RCLCPP_WARN(get_logger(),
                        "[RETARGET REVERSAL] generation=%lu policy=%s; %s",
                        task.pose_target.generation,
                        fr10_motion::poseExecutionPolicyName(task.policy),
                        parameterized.message.c_str());
            continue;
          }
          stop_reason_.store(StopReason::IK_FAILURE);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: TIME_PARAMETERIZATION] "
                       "generation=%lu reason=%s",
                       task.pose_target.generation,
                       parameterized.message.c_str());
          continue;
        }
        result.timing_diagnostics = parameterized.diagnostics;

        bool trajectory_valid = true;
        fr10_motion::KinematicState validation_seed = seed;
        for (const auto &point : parameterized.trajectory.points) {
          if (stop_requested_.load() ||
              task.sequence != kinematics_task_sequence_.load()) {
            trajectory_valid = false;
            rejection_reason = "trajectory FK validation canceled";
            break;
          }
          const auto state_validation =
              moveit_ik_provider_->validate(point.joint_state,
                                             validation_seed);
          if (!state_validation.success) {
            trajectory_valid = false;
            rejection_reason = state_validation.message;
            break;
          }
          fr10_motion::Pose reconstructed;
          if (!moveit_ik_provider_->forward(point.joint_state, reconstructed,
                                            &rejection_reason)) {
            trajectory_valid = false;
            break;
          }
          double squared_position_error = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double delta = reconstructed.position_m[axis] -
                                 point.desired_pose.position_m[axis];
            squared_position_error += delta * delta;
          }
          const double position_error = std::sqrt(squared_position_error);
          const double orientation_error =
              fr10_motion::math::quaternionAngularDistance(
                  reconstructed.orientation_quat,
                  point.desired_pose.orientation_quat);
          const double path_deviation =
              fr10_motion::CartesianPath::translationDeviation(
                  point.desired_pose, point.desired_translation_tangent,
                  reconstructed);
          result.planned_max_position_error_m =
              std::max(result.planned_max_position_error_m, position_error);
          result.planned_max_orientation_error_rad = std::max(
              result.planned_max_orientation_error_rad, orientation_error);
          result.planned_max_path_deviation_m =
              std::max(result.planned_max_path_deviation_m, path_deviation);
          // FREE_SPACE is a joint-space route: the independently sampled
          // Cartesian reference is diagnostic only and must not turn that
          // mode back into a straight-line Cartesian constraint.  Every
          // sample is still checked above for joint limits, singularity
          // barriers, and PlanningScene collision.  INSERTION alone enforces
          // the strict TCP/orientation/perpendicular invariant.
          const bool strict_cartesian_trace =
              task.policy == fr10_motion::PoseExecutionPolicy::INSERTION;
          if (!std::isfinite(position_error) ||
              !std::isfinite(orientation_error) ||
              !std::isfinite(path_deviation) ||
              (strict_cartesian_trace &&
               (position_error > params_.cartesian_position_tolerance_m ||
                orientation_error > params_.cartesian_angular_tolerance_rad ||
                path_deviation >
                    params_.cartesian_path_deviation_tolerance_m))) {
            trajectory_valid = false;
            rejection_reason =
                "time-parameterized FK trace exceeds Cartesian tolerance";
            break;
          }
          validation_seed = point.joint_state;
        }
        if (!trajectory_valid) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          stop_reason_.store(StopReason::PATH_DEVIATION);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(
              get_logger(),
              "[TARGET REJECTED: PATH_DEVIATION] generation=%lu reason=%s "
              "position=%.6f orientation=%.6f perpendicular=%.6f",
              task.pose_target.generation, rejection_reason.c_str(),
              result.planned_max_position_error_m,
              result.planned_max_orientation_error_rad,
              result.planned_max_path_deviation_m);
          continue;
        }
        result.trajectory_duration_s = parameterized.duration_s;
        auto raw_trajectory =
            std::make_shared<fr10_motion::CartesianJointTrajectory>(
                std::move(parameterized.trajectory));
        raw_trajectory->planning_timestamp_s = now().seconds();
        raw_trajectory->seed_joint_state = seed;
        result.pose_trajectory =
            motion_executor_->preparePoseTrajectory(std::move(raw_trajectory));
        if (!result.pose_trajectory) {
          stop_reason_.store(StopReason::PATH_DEVIATION);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(
              get_logger(),
              "[TARGET REJECTED: TRAJECTORY_VALIDATION] generation=%lu",
              task.pose_target.generation);
          continue;
        }
        result.joint_target.position_rad =
            continuous.joint_path.back().position_rad;
        result.joint_target.timestamp_s = task.pose_target.timestamp_s;
        const auto &active_mode_config =
            params_.path_time_policies[policyIndex(task.policy)];
        result.joint_target.velocity_scale =
            std::min({active_mode_config.joint_velocity_scale,
                      active_mode_config.linear_scale,
                      active_mode_config.angular_scale});
        result.joint_target.acceleration_scale =
            active_mode_config.joint_acceleration_scale;
        result.joint_target.jerk_scale = active_mode_config.joint_jerk_scale;
      } else {
        fr10_motion::KinematicState target;
        target.position_rad = task.joint_target.position_rad;
        const fr10_motion::IKResult validation =
            path_ik_provider_->validate(target, seed);
        if (!validation.success) {
          stop_reason_.store(StopReason::SINGULARITY);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: SINGULARITY] sequence=%lu reason=%s",
                       task.sequence, validation.message.c_str());
          continue;
        }
        result.manipulability = validation.manipulability;
        result.condition_number = validation.condition_number;
        // Validate the complete Ruckig stream and execute this exact buffer;
        // validating only the endpoint misses intermediate collisions.
        auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
        trajectory->generation = task.sequence;
        trajectory->seed_joint_state = seed;
        trajectory->planning_timestamp_s = now().seconds();
        fr10_motion::JointOtg planner(params_.nominal_period_s,
            params_.controller_config.limits,
            params_.controller_config.max_joint_reference_lead_rad);
        planner.reset(seed.position_rad, seed.velocity_rad_s, seed.acceleration_rad_s2);
        planner.submitTarget(task.joint_target);
        fr10_motion::CartesianTrajectoryPoint point;
        point.joint_state = seed;
        trajectory->points.push_back(point);
        bool path_valid = true;
        auto previous = seed;
        constexpr int kMaximumJointPlanSamples = 15000;
        for (int sample = 1; sample <= kMaximumJointPlanSamples; ++sample) {
          if (task.sequence != kinematics_task_sequence_.load() || stop_requested_.load() ||
              !planner.step(task.seed, point.joint_state.position_rad,
                            point.joint_state.velocity_rad_s, point.joint_state.acceleration_rad_s2)) {
            path_valid = false;
            break;
          }
          const auto checked = path_ik_provider_->validate(point.joint_state, previous);
          if (!checked.success) {
            RCLCPP_ERROR(get_logger(), "[JOINT PATH REJECTED] sequence=%lu sample=%d reason=%s",
                        task.sequence, sample, checked.message.c_str());
            path_valid = false;
            break;
          }
          previous = point.joint_state;
          point.time_from_start_s = sample * params_.nominal_period_s;
          trajectory->points.push_back(point);
          if (planner.isFinished()) { break; }
        }
        if (!path_valid || !planner.isFinished()) { continue; }
        for (std::size_t i = 0; i < trajectory->points.size(); ++i) {
          trajectory->points[i].path_parameter = double(i) / double(trajectory->points.size() - 1);
        }
        result.trajectory_duration_s = trajectory->points.back().time_from_start_s;
        result.pose_trajectory = motion_executor_->prepareJointTrajectory(trajectory, result.joint_target);
        if (!result.pose_trajectory) { continue; }
      }

      if (task.sequence != kinematics_task_sequence_.load()) {
        continue;
      }
      if (!publishKinematicsResult(result)) {
        RCLCPP_WARN(get_logger(),
                    "Kinematics result mailbox busy; dropping sequence=%lu",
                    result.sequence);
        continue;
      }

      if (task.control_mode == fr10_motion::ControlMode::POSE) {
        RCLCPP_INFO(
            get_logger(),
            "[NEW_GOAL] Pose gen=%lu policy=%s validated with %zu IK "
            "waypoints and %zu "
            "timed samples (%.3f s); condition=%.2f path_error=%.6f m route=%s "
            "clearance=%.3f m",
            task.pose_target.generation,
            fr10_motion::poseExecutionPolicyName(task.policy),
            result.ik_waypoint_count,
            result.pose_trajectory
                ? result.pose_trajectory->trajectory().points.size()
                : 0,
            result.trajectory_duration_s, result.condition_number,
            result.planned_max_path_deviation_m,
            task.policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE
                ? "FREE_SPACE_JOINT"
                : (result.used_alternate_orientation
                       ? "ALTERNATE_ORIENTATION"
                       : (result.used_joint_space_detour
                              ? "JOINT_SPACE_DETOUR"
                              : (result.used_kinematic_detour
                                     ? "KINEMATIC_DETOUR"
                                     : "DIRECT"))),
            result.detour_clearance_m);
      } else {
        RCLCPP_INFO(get_logger(),
                    "[NEW_GOAL] Joint rev=%lu validated; manipulability=%.6g "
                    "condition=%.2f",
                    result.joint_target.revision, result.manipulability,
                    result.condition_number);
      }
    }
  }

  void consumeKinematicsResult() {
    KinematicsResult result;
    if (!kinematics_result_buffer_.read(result) ||
        result.sequence == consumed_kinematics_result_sequence_ ||
        result.sequence != kinematics_task_sequence_.load()) {
      return;
    }
    consumed_kinematics_result_sequence_ = result.sequence;

    const uint64_t current_scene_revision =
        scene_revision_.load(std::memory_order_acquire);
    if (params_.collision_checking_enabled &&
        result.scene_revision != current_scene_revision) {
      fr10_motion::RobotState live_seed;
      if (!feedbackSnapshot(live_seed)) {
        RCLCPP_WARN(get_logger(),
                    "[STALE SCENE] feedback unavailable; result sequence=%lu "
                    "discarded",
                    result.sequence);
        return;
      }
      fr10_motion::MotionCommand live_reference;
      latest_command_buffer_.read(live_reference);
      if (live_reference.type ==
          fr10_motion::CommandType::JOINT_POSITION) {
        live_seed.joint_position_rad = live_reference.joint_position_rad;
        live_seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
        live_seed.joint_acceleration_rad_s2 =
            live_reference.joint_acceleration_rad_s2;
      }
      KinematicsTask retry;
      retry.control_mode = result.control_mode;
      retry.joint_target = result.joint_target;
      retry.pose_target = result.pose_target;
      retry.policy = result.policy;
      retry.seed = live_seed;
      retry.replan_attempt = result.replan_attempt + 1;
      retry.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
      RCLCPP_WARN(get_logger(),
                  "[STALE SCENE] planned=%lu current=%lu sequence=%lu; "
                  "replanning latest target",
                  result.scene_revision, current_scene_revision,
                  result.sequence);
      queueKinematicsTask(retry);
      return;
    }

    if (result.requires_controlled_stop_replan) {
      KinematicsTask retry;
      retry.control_mode = fr10_motion::ControlMode::POSE;
      retry.pose_target = result.pose_target;
      retry.policy = result.policy;
      retry.replan_attempt = result.replan_attempt + 1;
      {
        std::lock_guard<std::mutex> lock(reversal_mutex_);
        post_stop_replan_task_ = retry;
      }
      RCLCPP_WARN(get_logger(),
                  "[RETARGET REVERSAL] generation=%lu requesting jerk-limited "
                  "controlled stop before zero-velocity replan",
                  result.pose_target.generation);
      requestControlledStop(StopReason::RETARGET_REVERSAL);
      return;
    }

    if (result.control_mode == fr10_motion::ControlMode::JOINT) {
      if (accepted_target_writer_.have_joint_target &&
          motion_executor_->status().outcome != fr10_motion::MotionOutcome::STOPPED_BEFORE_GOAL) {
        double max_change = 0.0;
        for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
          max_change = std::max(max_change,
                                std::abs(result.joint_target.position_rad[i] -
                                         accepted_target_writer_.joint_target
                                             .position_rad[i]));
        }
        if (max_change <= params_.joint_change_tolerance_rad) {
          return;
        }
      }
    }

    fr10_motion::PoseActivationResult pose_activation =
        fr10_motion::PoseActivationResult::REJECTED;
    const bool accepted =
        (pose_activation =
                   motion_executor_->activatePreparedPoseTrajectory(
                       result.pose_trajectory)) ==
                  fr10_motion::PoseActivationResult::ACCEPTED;
    if (!accepted) {
      if (pose_activation == fr10_motion::PoseActivationResult::STALE &&
          result.replan_attempt < 3) {
        fr10_motion::RobotState live_seed;
        if (!feedbackSnapshot(live_seed)) {
          RCLCPP_WARN(get_logger(),
                      "[STALE TRAJECTORY] feedback unavailable; retaining "
                      "previous valid motion state");
          return;
        }
        fr10_motion::MotionCommand live_reference;
        latest_command_buffer_.read(live_reference);
        if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
          live_seed.joint_position_rad = live_reference.joint_position_rad;
          live_seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
          live_seed.joint_acceleration_rad_s2 =
              live_reference.joint_acceleration_rad_s2;
        }
        KinematicsTask retry;
        retry.control_mode = result.control_mode;
        retry.joint_target = result.joint_target;
        retry.pose_target = result.pose_target;
        retry.policy = result.policy;
        retry.seed = live_seed;
        retry.replan_attempt = result.replan_attempt + 1;
        retry.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
        RCLCPP_WARN(
            get_logger(),
            "[STALE TRAJECTORY] generation=%lu plan_stamp=%.6f "
            "activation seed diverged from live state; replan=%u/3",
            result.pose_target.generation,
            result.pose_trajectory
                ? result.pose_trajectory->trajectory().planning_timestamp_s
                : 0.0,
            retry.replan_attempt);
        queueKinematicsTask(retry);
        return;
      }
      stop_reason_.store(StopReason::SINGULARITY);
      pending_event_.store(StopReason::SINGULARITY);
      return;
    }

    if (result.control_mode == fr10_motion::ControlMode::JOINT) {
      accepted_target_writer_.joint_target = result.joint_target;
      accepted_target_writer_.have_joint_target = true;
    } else {
      accepted_target_writer_.cartesian_target = result.pose_target;
      accepted_target_writer_.have_cartesian_target = true;
    }
    accepted_target_buffer_.write(accepted_target_writer_);
    active_control_mode_.store(result.control_mode);
    if (result.control_mode == fr10_motion::ControlMode::POSE) {
      active_pose_policy_.store(result.policy);
    }
    stop_reason_.store(StopReason::NONE);
    active_scene_revision_.store(result.scene_revision,
                                 std::memory_order_release);
  }

  void queuePostStopReplanIfReady(const fr10_motion::MotionStatus &status,
                                  const fr10_motion::RobotState &feedback,
                                  const fr10_motion::MotionCommand &command) {
    if (status.state != fr10_motion::MotionState::HOLDING) {
      return;
    }
    std::optional<KinematicsTask> retry;
    {
      std::lock_guard<std::mutex> lock(reversal_mutex_);
      if (!post_stop_replan_task_) {
        return;
      }
      retry = std::move(post_stop_replan_task_);
      post_stop_replan_task_.reset();
    }
    retry->seed = feedback;
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      retry->seed.joint_position_rad = command.joint_position_rad;
      retry->seed.joint_velocity_rad_s = command.joint_velocity_rad_s;
      retry->seed.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
    } else {
      retry->seed.joint_velocity_rad_s.fill(0.0);
      retry->seed.joint_acceleration_rad_s2.fill(0.0);
    }
    retry->sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    stop_reason_.store(StopReason::NONE);
    RCLCPP_INFO(get_logger(),
                "[RETARGET REPLAN] generation=%lu policy=%s from "
                "controlled-stop "
                "boundary",
                retry->pose_target.generation,
                fr10_motion::poseExecutionPolicyName(retry->policy));
    queueKinematicsTask(*retry);
  }

  void jointTargetCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr &message) {
    if (stop_requested_.load() || !motion_executor_->initialized() ||
        !backend_ || !backend_->isReady() ||
        message->name.size() != fr10_motion::kJointCount ||
        message->position.size() != fr10_motion::kJointCount) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Joint target rejected: ros2_control not ready, feedback "
          "not synchronized, or six names/positions not provided");
      return;
    }

    if (!updateJointIndexMap(*message)) {
      RCLCPP_WARN(
          get_logger(),
          "Joint target rejected: names must contain j1..j6 exactly once");
      return;
    }

    fr10_motion::JointTarget target;
    for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
      const std::size_t index = joint_command_index_map_[i];
      const double input_position = message->position[index];
      if (!std::isfinite(input_position)) {
        RCLCPP_WARN(get_logger(),
                    "Joint target rejected: joint values must be finite");
        return;
      }
      target.position_rad[i] = params_.joint_input_unit == "degrees"
                                   ? input_position * fr10_motion::kDegToRad
                                   : input_position;
    }
    target.velocity_scale = params_.joint_velocity_scale;
    target.acceleration_scale = params_.joint_acceleration_scale;
    target.jerk_scale = params_.joint_jerk_scale;
    target.timestamp_s = now().seconds();

    fr10_motion::RobotState seed;
    if (!feedbackSnapshot(seed)) {
      RCLCPP_WARN(get_logger(),
                  "Joint target rejected: no valid feedback seed");
      return;
    }

    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::JOINT;
    task.joint_target = target;
    task.joint_target.revision = joint_revision_.fetch_add(1) + 1;
    task.seed = seed;
    fr10_motion::MotionCommand reference;
    if (latest_command_buffer_.read(reference) &&
        reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 = reference.joint_acceleration_rad_s2;
    }
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    queueKinematicsTask(task);
  }

  void poseTargetCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message) {
    if (shutdown_in_progress_.load() || stop_latch_.load() != StopReason::NONE ||
        motion_executor_->status().state == fr10_motion::MotionState::STOPPING) { return; }
    if (message->header.frame_id != params_.base_frame) {
      RCLCPP_WARN(get_logger(),
                  "Insertion target rejected: frame '%s' must be '%s'",
                  message->header.frame_id.c_str(), params_.base_frame.c_str());
      return;
    }
    const rclcpp::Time receive_time = now();
    const rclcpp::Time measurement_time(message->header.stamp,
                                        get_clock()->get_clock_type());
    if (measurement_time.nanoseconds() != 0) {
      const double age_s = (receive_time - measurement_time).seconds();
      if (!std::isfinite(age_s) || age_s < -0.10 ||
          age_s > params_.max_measurement_age_s) {
        RCLCPP_WARN(get_logger(),
                    "Pose target rejected: stale/future timestamp age=%.3f s",
                    age_s);
        return;
      }
    }
    fr10_motion::Pose target_pose;
    target_pose.position_m = {message->pose.position.x,
                              message->pose.position.y,
                              message->pose.position.z};
    target_pose.orientation_quat = {
        message->pose.orientation.x, message->pose.orientation.y,
        message->pose.orientation.z, message->pose.orientation.w};
    if (!std::all_of(target_pose.position_m.begin(),
                     target_pose.position_m.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !fr10_motion::math::normalizeQuaternion(target_pose.orientation_quat)) {
      RCLCPP_WARN(get_logger(),
                  "Insertion target rejected: invalid pose/quaternion");
      return;
    }
    const auto staging =
        fr10_motion::PreInsertPlanner::plan(target_pose, params_.pre_insert_config);
    const auto insertion =
        staging.success
            ? fr10_motion::LinearInsertPlanner::plan(
                  staging.pre_insert_pose, target_pose, params_.linear_insert_config)
            : fr10_motion::LinearInsertPlan{};
    if (!staging.success || !insertion.success) {
      RCLCPP_ERROR(get_logger(), "[INSERTION REJECTED: GEOMETRY] %s",
                   staging.success ? insertion.message.c_str()
                                   : staging.message.c_str());
      return;
    }

    fr10_motion::CartesianTarget insertion_target =
        fr10_motion::CartesianTarget::fromPose(target_pose);
    const auto &policy_config = params_.path_time_policies[policyIndex(
        fr10_motion::PoseExecutionPolicy::INSERTION)];
    insertion_target.linear_speed_scale = policy_config.linear_scale;
    insertion_target.angular_speed_scale = policy_config.angular_scale;
    insertion_target.timestamp_s = now().seconds();

    fr10_motion::RobotState live_seed;
    fr10_motion::Pose live_tcp;
    if (feedbackSnapshot(live_seed)) {
      live_tcp = {live_seed.tcp_position_m, live_seed.tcp_orientation_quat};
      double pre_insert_error_squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double error = live_tcp.position_m[axis] -
                             staging.pre_insert_pose.position_m[axis];
        pre_insert_error_squared += error * error;
      }
      const double orientation_error =
          fr10_motion::math::quaternionAngularDistance(
              live_tcp.orientation_quat,
              staging.pre_insert_pose.orientation_quat);
      if (std::sqrt(pre_insert_error_squared) <=
              params_.linear_insert_config.position_tolerance_m &&
          orientation_error <=
              params_.linear_insert_config.orientation_tolerance_rad) {
        fr10_motion::MotionCommand live_reference;
        latest_command_buffer_.read(live_reference);
        if (queueStrictInsertionTarget(insertion_target,
                                       staging.pre_insert_pose, target_pose,
                                       live_seed, live_reference)) {
          display_pose_goal_buffer_.write(
              DisplayPoseGoalSnapshot{target_pose, true});
          std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
          pending_insertion_sequence_.reset();
          last_insertion_target_ = insertion_target;
          RCLCPP_INFO(get_logger(),
                      "[MODE TRANSITION] already at PRE_INSERT; entering "
                      "INSERTION policy directly");
        }
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      const fr10_motion::CartesianTarget *duplicate_reference = nullptr;
      if (pending_insertion_sequence_) {
        duplicate_reference = &pending_insertion_sequence_->insertion_target;
      } else if (last_insertion_target_) {
        duplicate_reference = &*last_insertion_target_;
      }
      if (duplicate_reference != nullptr) {
        double squared_change = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double delta = insertion_target.position_m[axis] -
                               duplicate_reference->position_m[axis];
          squared_change += delta * delta;
        }
        const double angular_change =
            fr10_motion::math::quaternionAngularDistance(
                insertion_target.orientation_quat,
                duplicate_reference->orientation_quat);
        if (std::sqrt(squared_change) <= params_.pose_position_change_tolerance_m &&
            angular_change <= params_.pose_angular_change_tolerance_rad) {
          return;
        }
      }
    }

    auto staging_message =
        std::make_shared<geometry_msgs::msg::PoseStamped>(*message);
    staging_message->pose.position.x = staging.pre_insert_pose.position_m[0];
    staging_message->pose.position.y = staging.pre_insert_pose.position_m[1];
    staging_message->pose.position.z = staging.pre_insert_pose.position_m[2];
    staging_message->pose.orientation.x =
        staging.pre_insert_pose.orientation_quat[0];
    staging_message->pose.orientation.y =
        staging.pre_insert_pose.orientation_quat[1];
    staging_message->pose.orientation.z =
        staging.pre_insert_pose.orientation_quat[2];
    staging_message->pose.orientation.w =
        staging.pre_insert_pose.orientation_quat[3];
    const auto staging_generation = queuePoseTarget(
        staging_message, fr10_motion::PoseExecutionPolicy::FREE_SPACE, true);
    if (!staging_generation) {
      RCLCPP_WARN(get_logger(), "Insertion staging target was not queued");
      return;
    }
    display_pose_goal_buffer_.write(DisplayPoseGoalSnapshot{target_pose, true});
    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      pending_insertion_sequence_ = PendingInsertionSequence{
          insertion_target, staging.pre_insert_pose, *staging_generation};
    }
    RCLCPP_INFO(
        get_logger(),
        "[POSE POLICY TRANSITION ARMED] FREE_SPACE -> PRE_INSERT (%.1f mm) "
        "-> INSERTION",
        staging.distance_m * 1000.0);
  }

  std::optional<uint64_t> queuePoseTarget(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message,
      fr10_motion::PoseExecutionPolicy policy,
      bool preserve_orientation = false) {
    if (stop_requested_.load() || !motion_executor_->initialized() ||
        !backend_ || !backend_->isReady()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Cartesian target rejected: ros2_control command "
                           "consumer is not ready");
      return std::nullopt;
    }
    if (message->header.frame_id != params_.base_frame) {
      RCLCPP_WARN(get_logger(),
                  "Cartesian target rejected: frame '%s' must be '%s'",
                  message->header.frame_id.c_str(), params_.base_frame.c_str());
      return std::nullopt;
    }
    const rclcpp::Time receive_time = now();
    const rclcpp::Time measurement_time(message->header.stamp,
                                        get_clock()->get_clock_type());
    if (measurement_time.nanoseconds() != 0) {
      const double age_s = (receive_time - measurement_time).seconds();
      if (!std::isfinite(age_s) || age_s < -0.10 ||
          age_s > params_.max_measurement_age_s) {
        RCLCPP_WARN(
            get_logger(),
            "Cartesian target rejected: stale/future timestamp age=%.3f s",
            age_s);
        return std::nullopt;
      }
    }

    fr10_motion::CartesianTarget target;
    target.position_m = {message->pose.position.x, message->pose.position.y,
                         message->pose.position.z};
    target.orientation_quat = {
        message->pose.orientation.x, message->pose.orientation.y,
        message->pose.orientation.z, message->pose.orientation.w};
    if (!std::all_of(target.position_m.begin(), target.position_m.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !fr10_motion::math::normalizeQuaternion(target.orientation_quat)) {
      RCLCPP_WARN(get_logger(),
                  "Cartesian target rejected: non-finite pose/zero quaternion");
      return std::nullopt;
    }

    fr10_motion::RobotState seed;
    if (!feedbackSnapshot(seed)) {
      RCLCPP_WARN(get_logger(),
                  "Cartesian target rejected: no valid feedback seed");
      return std::nullopt;
    }

    const auto &active_policy_config =
        params_.path_time_policies[policyIndex(policy)];
    target.linear_speed_scale = active_policy_config.linear_scale;
    target.angular_speed_scale = active_policy_config.angular_scale;
    target.timestamp_s = receive_time.seconds();

    fr10_motion::Pose tcp_feedback;
    tcp_feedback = {seed.tcp_position_m, seed.tcp_orientation_quat};
    {
      const double orientation_deviation =
          fr10_motion::math::quaternionAngularDistance(
              target.orientation_quat, tcp_feedback.orientation_quat);
      if (params_.lock_orientation && !preserve_orientation) {
        target.orientation_quat = tcp_feedback.orientation_quat;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Cartesian orientation locked at current TCP orientation "
            "(requested deviation %.1f deg)",
            orientation_deviation * fr10_motion::kRadToDeg);
      } else if (orientation_deviation > params_.max_angular_deviation_rad) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[LARGE ORIENTATION] Planning %.1f deg with dense quaternion "
            "SLERP; orientation is not locked",
            orientation_deviation * fr10_motion::kRadToDeg);
      }
    }

    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      double position_change = std::numeric_limits<double>::infinity();
      double angular_change = std::numeric_limits<double>::infinity();
      if (have_queued_cartesian_target_) {
        double squared_change = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
          const double delta = target.position_m[i] -
                               last_queued_cartesian_target_.position_m[i];
          squared_change += delta * delta;
        }
        position_change = std::sqrt(squared_change);
        angular_change = fr10_motion::math::quaternionAngularDistance(
            target.orientation_quat,
            last_queued_cartesian_target_.orientation_quat);
      }
      if (position_change <= params_.pose_position_change_tolerance_m &&
          angular_change <= params_.pose_angular_change_tolerance_rad) {
        return std::nullopt;
      }
    }

    target.generation = cartesian_generation_.fetch_add(1) + 1;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      last_queued_cartesian_target_ = target;
      have_queued_cartesian_target_ = true;
    }
    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::POSE;
    task.pose_target = target;
    task.policy = policy;
    task.seed = seed;
    fr10_motion::MotionCommand live_reference;
    latest_command_buffer_.read(live_reference);
    if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = live_reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 =
          live_reference.joint_acceleration_rad_s2;
    }
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    queueKinematicsTask(task);
    return target.generation;
  }

  bool
  queueStrictInsertionTarget(fr10_motion::CartesianTarget target,
                             const fr10_motion::Pose &expected_path_start,
                             const fr10_motion::Pose &expected_path_goal,
                             const fr10_motion::RobotState &seed,
                             const fr10_motion::MotionCommand &live_reference) {
    fr10_motion::Pose current_pose;
    current_pose.position_m = seed.tcp_position_m;
    current_pose.orientation_quat = seed.tcp_orientation_quat;
    fr10_motion::KinematicState measured_joints;
    measured_joints.position_rad = seed.joint_position_rad;
    if (!analytical_ik_provider_->forward(measured_joints, current_pose)) { return false; }
    const fr10_motion::Pose target_pose = target.toPose();
    const auto straight_segment = fr10_motion::LinearInsertPlanner::plan(
        current_pose, target_pose, params_.linear_insert_config);
    const double axis_deviation =
        fr10_motion::CartesianPath::translationDeviation(
            expected_path_start, expected_path_goal, current_pose);
    std::array<double, 3> expected_direction{};
    std::array<double, 3> remaining{};
    double forward_projection = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      expected_direction[axis] = expected_path_goal.position_m[axis] -
                                 expected_path_start.position_m[axis];
      remaining[axis] = target.position_m[axis] - current_pose.position_m[axis];
      forward_projection += expected_direction[axis] * remaining[axis];
    }
    if (!straight_segment.success || !std::isfinite(axis_deviation) ||
        axis_deviation > params_.linear_insert_config.perpendicular_tolerance_m ||
        forward_projection < -1e-9) {
      RCLCPP_ERROR(get_logger(),
                   "[INSERTION REJECTED: AXIS_INVARIANT] deviation=%.3f mm "
                   "forward_projection=%.6g reason=%s; reposition PRE_INSERT",
                   axis_deviation * 1000.0, forward_projection,
                   straight_segment.message.c_str());
      return false;
    }

    target.generation = cartesian_generation_.fetch_add(1) + 1;
    target.timestamp_s = now().seconds();
    const auto &policy_config = params_.path_time_policies[policyIndex(
        fr10_motion::PoseExecutionPolicy::INSERTION)];
    target.linear_speed_scale = policy_config.linear_scale;
    target.angular_speed_scale = policy_config.angular_scale;
    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::POSE;
    task.pose_target = target;
    task.policy = fr10_motion::PoseExecutionPolicy::INSERTION;
    task.seed = seed;
    if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = live_reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 =
          live_reference.joint_acceleration_rad_s2;
    }
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      last_queued_cartesian_target_ = target;
      have_queued_cartesian_target_ = true;
    }
    queueKinematicsTask(task);
    RCLCPP_INFO(
        get_logger(),
        "[POSE POLICY QUEUED] INSERTION generation=%lu distance=%.1f mm "
        "axis_deviation=%.3f mm",
        target.generation, straight_segment.distance_m * 1000.0,
        axis_deviation * 1000.0);
    return true;
  }

  void
  queueInsertionAfterStagingIfReady(const fr10_motion::MotionStatus &status,
                                    const fr10_motion::RobotState &feedback,
                                    const fr10_motion::MotionCommand &command) {
    if (status.state != fr10_motion::MotionState::HOLDING ||
        status.outcome != fr10_motion::MotionOutcome::GOAL_REACHED ||
        !status.in_goal_tolerance) {
      return;
    }
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      constexpr double kBoundaryVelocityToleranceRadS = 1e-5;
      constexpr double kBoundaryAccelerationToleranceRadS2 = 1e-4;
      for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
        if (std::abs(command.joint_velocity_rad_s[joint]) >
                kBoundaryVelocityToleranceRadS ||
            std::abs(command.joint_acceleration_rad_s2[joint]) >
                kBoundaryAccelerationToleranceRadS2) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "PRE_INSERT HOLDING boundary is not settled: waiting for "
              "qdot/qddot -> 0 before INSERTION");
          return;
        }
      }
    }
    std::optional<PendingInsertionSequence> pending;
    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      if (!pending_insertion_sequence_ ||
          pending_insertion_sequence_->staging_generation !=
              status.active_generation) {
        return;
      }
      pending = pending_insertion_sequence_;
    }

    // A completed reference is not enough to authorize INSERTION. The
    // measured robot must be at the PRE_INSERT boundary on this fresh
    // feedback sample; otherwise HOLDING is allowed to let the servo settle.
    fr10_motion::KinematicState measured_joints;
    measured_joints.position_rad = feedback.joint_position_rad;
    fr10_motion::Pose measured_tcp;
    if (!feedback.valid || !analytical_ik_provider_ ||
        !analytical_ik_provider_->forward(measured_joints, measured_tcp)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PRE_INSERT boundary waiting for valid measured TCP feedback");
      return;
    }
    double boundary_error_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double error = measured_tcp.position_m[axis] -
                           pending->pre_insert_pose.position_m[axis];
      boundary_error_squared += error * error;
    }
    const double boundary_position_error = std::sqrt(boundary_error_squared);
    const double boundary_orientation_error =
        fr10_motion::math::quaternionAngularDistance(
            measured_tcp.orientation_quat,
            pending->pre_insert_pose.orientation_quat);
    if (boundary_position_error >
            params_.linear_insert_config.position_tolerance_m ||
        boundary_orientation_error >
            params_.linear_insert_config.orientation_tolerance_rad) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PRE_INSERT measured boundary not reached: position=%.3f mm "
          "orientation=%.3f deg",
          boundary_position_error * 1000.0,
          boundary_orientation_error * fr10_motion::kRadToDeg);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      if (!pending_insertion_sequence_ ||
          pending_insertion_sequence_->staging_generation !=
              status.active_generation) {
        return;
      }
      pending_insertion_sequence_.reset();
    }
    const fr10_motion::Pose target_pose =
        pending->insertion_target.toPose();
    if (queueStrictInsertionTarget(pending->insertion_target,
                                   pending->pre_insert_pose, target_pose,
                                   feedback, command)) {
      {
        std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
        last_insertion_target_ = pending->insertion_target;
      }
      stop_reason_.store(StopReason::NONE);
      RCLCPP_INFO(get_logger(), "[POSE POLICY TRANSITION] PRE_INSERT reached; "
                                "FREE_SPACE -> INSERTION");
    }
  }

  void
  retractCallback(const std_msgs::msg::Empty::ConstSharedPtr & /*message*/) {
    std::optional<fr10_motion::CartesianTarget> socket_target;
    {
      std::lock_guard<std::mutex> lock(insertion_sequence_mutex_);
      pending_insertion_sequence_.reset();
      socket_target = last_insertion_target_;
    }
    if (!socket_target) {
      RCLCPP_WARN(get_logger(),
                  "Retract rejected: no validated insertion target retained");
      return;
    }
    const fr10_motion::Pose socket_pose = socket_target->toPose();
    const auto staging =
        fr10_motion::PreInsertPlanner::plan(socket_pose, params_.pre_insert_config);
    fr10_motion::RobotState seed;
    if (!staging.success || !feedbackSnapshot(seed)) {
      RCLCPP_ERROR(get_logger(), "Retract rejected: %s",
                   staging.success ? "feedback unavailable"
                                   : staging.message.c_str());
      return;
    }
    fr10_motion::CartesianTarget retract_target =
        fr10_motion::CartesianTarget::fromPose(staging.pre_insert_pose);
    fr10_motion::MotionCommand live_reference;
    latest_command_buffer_.read(live_reference);
    if (queueStrictInsertionTarget(retract_target, socket_pose,
                                   staging.pre_insert_pose, seed,
                                   live_reference)) {
      display_pose_goal_buffer_.write(
          DisplayPoseGoalSnapshot{staging.pre_insert_pose, true});
      RCLCPP_INFO(get_logger(),
                  "[PURE LINEAR RETRACT] INSERTION policy target=PRE_INSERT");
    }
  }

  void controlTick() {
    if (stop_requested_.load()) {
      return;
    }
    const auto tick_start = std::chrono::steady_clock::now();
    const double measured_period_s =
        std::chrono::duration<double>(tick_start - last_tick_).count();
    last_tick_ = tick_start;

    fr10_motion::RobotState feedback;
    if (!backend_ || !backend_->readState(feedback)) {
      if (params_.backend_name == "ros_control" && !motion_executor_->initialized()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Waiting for the first complete ros_control "
                             "/joint_states sample");
        return;
      }
      ++consecutive_feedback_failures_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Robot feedback read failed (%d consecutive)",
                            consecutive_feedback_failures_);
      if (consecutive_feedback_failures_ >= 1) {
        requestStopAll(StopReason::COMM_FAILURE);
      }
      return;
    }
    consecutive_feedback_failures_ = 0;
    if (backend_->isReady() && !backend_ready_announced_.exchange(true)) {
      RCLCPP_INFO(
          get_logger(),
          "[READY] fr10_pose_controller ready at %.1f Hz; waiting for a target",
          params_.controller_config.control_rate_hz);
    }
    feedback_buffer_.write(feedback);
    const auto pre_update_status = motion_executor_->status();
    if (params_.collision_checking_enabled &&
        pre_update_status.state == fr10_motion::MotionState::TRACKING &&
        active_scene_revision_.load(std::memory_order_acquire) !=
            scene_revision_.load(std::memory_order_acquire)) {
      StopReason expected = StopReason::NONE;
      if (stop_latch_.compare_exchange_strong(expected,
                                               StopReason::SCENE_CHANGED)) {
        RCLCPP_WARN(get_logger(),
                    "[SCENE CHANGED] active=%lu current=%lu; requesting "
                    "controlled stop",
                    active_scene_revision_.load(std::memory_order_acquire),
                    scene_revision_.load(std::memory_order_acquire));
      }
    }
    const auto stop_request = stop_latch_.exchange(StopReason::NONE);
    if (stop_request != StopReason::NONE) {
      if (stop_request != StopReason::PATH_DEVIATION ||
          path_stop_generation_.exchange(0) ==
              motion_executor_->status().active_generation) {
        requestControlledStop(stop_request);
      }
    } else if (!shutdown_in_progress_.load()) {
      consumeKinematicsResult();
    }

    const auto status = motion_executor_->update(feedback, measured_period_s);
    const auto command = motion_executor_->command();
    if (status.lag_event.sequence > handled_lag_event_) {
      handled_lag_event_ = status.lag_event.sequence;
      // Invalidates staging/reversal/worker results as well as the active goal.
      // Executor stop() is idempotent and cannot restart the braking profile.
      requestControlledStop(StopReason::REFERENCE_LAG);
    }
    queuePostStopReplanIfReady(status, feedback, command);
    queueInsertionAfterStagingIfReady(status, feedback, command);
    if (status.state == fr10_motion::MotionState::FAULT) {
      const StopReason fault_reason =
          stop_reason_.load() == StopReason::PATH_DEVIATION
              ? StopReason::PATH_DEVIATION
              : StopReason::HARDWARE_ALARM;
      RCLCPP_ERROR(
          get_logger(),
          "MotionExecutor FAULT reason=%s: feedback_valid=%s emergency=%s "
          "collision=%s robot_mode=%d main_error=%d sub_error=%d",
          stopReasonName(fault_reason), feedback.valid ? "true" : "false",
          feedback.emergency_stop ? "true" : "false",
          feedback.collision_detected ? "true" : "false", feedback.robot_mode,
          feedback.main_error_code, feedback.sub_error_code);
      requestStopAll(fault_reason);
      return;
    }

    if (command.type != fr10_motion::CommandType::NONE) {
      const bool command_ok = backend_ && backend_->writeCommand(command);
      if (!command_ok) {
        ++command_write_errors_;
        RCLCPP_ERROR(
            get_logger(),
            "Backend command write failed: type=%s sequence=%lu errors=%lu",
            fr10_motion::commandTypeName(command.type), command.sequence,
            command_write_errors_);
        requestStopAll(StopReason::COMM_FAILURE);
        return;
      }
      updateContinuityMetrics(command);
      motion_executor_->acknowledgeCommand(command.sequence);
      latest_command_buffer_.write(command);
    }

    ++loop_samples_;
    loop_elapsed_s_ += measured_period_s;
    const double absolute_jitter_ms =
        std::abs(measured_period_s - params_.nominal_period_s) * 1000.0;
    loop_abs_jitter_sum_ms_ += absolute_jitter_ms;
    max_abs_jitter_ms_ = std::max(max_abs_jitter_ms_, absolute_jitter_ms);
    max_cycle_period_ms_ =
        std::max(max_cycle_period_ms_, measured_period_s * 1000.0);
    DiagnosticsSnapshot snapshot;
    snapshot.status = status;
    snapshot.feedback = feedback;
    snapshot.command = command;
    snapshot.loop.samples = loop_samples_;
    snapshot.loop.elapsed_s = loop_elapsed_s_;
    snapshot.loop.mean_abs_jitter_ms =
        loop_abs_jitter_sum_ms_ / static_cast<double>(loop_samples_);
    snapshot.loop.max_abs_jitter_ms = max_abs_jitter_ms_;
    snapshot.loop.max_cycle_period_ms = max_cycle_period_ms_;
    snapshot.loop.max_delta_joint_velocity_rad_s =
        max_delta_joint_velocity_rad_s_;
    snapshot.loop.max_delta_linear_velocity_m_s =
        max_delta_linear_velocity_m_s_;
    snapshot.loop.command_write_errors = command_write_errors_;
    diagnostics_buffer_.write(snapshot);
    if (shutdown_in_progress_.load() &&
        (status.state == fr10_motion::MotionState::HOLDING ||
         status.state == fr10_motion::MotionState::IDLE)) {
      requestStopAll(StopReason::USER_INTERRUPT);
    }
  }

  void updateContinuityMetrics(const fr10_motion::MotionCommand &command) {
    if (have_previous_command_ && command.type == previous_command_.type) {
      if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
        for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
          max_delta_joint_velocity_rad_s_ =
              std::max(max_delta_joint_velocity_rad_s_,
                       std::abs(command.joint_velocity_rad_s[i] -
                                previous_command_.joint_velocity_rad_s[i]));
        }
      }
    }
    previous_command_ = command;
    have_previous_command_ = true;
  }

  void publishTcpPose(const fr10_motion::RobotState &feedback) {
    tcp_pose_message_.header.stamp = now();
    tcp_pose_message_.pose.position.x = feedback.tcp_position_m[0];
    tcp_pose_message_.pose.position.y = feedback.tcp_position_m[1];
    tcp_pose_message_.pose.position.z = feedback.tcp_position_m[2];
    tcp_pose_message_.pose.orientation.x = feedback.tcp_orientation_quat[0];
    tcp_pose_message_.pose.orientation.y = feedback.tcp_orientation_quat[1];
    tcp_pose_message_.pose.orientation.z = feedback.tcp_orientation_quat[2];
    tcp_pose_message_.pose.orientation.w = feedback.tcp_orientation_quat[3];
    tcp_pose_pub_->publish(tcp_pose_message_);
  }

  void pathMonitorTick() {
    const auto tick = std::chrono::steady_clock::now();
    const double dt =
        std::chrono::duration<double>(tick - path_monitor_last_tick_).count();
    path_monitor_last_tick_ = tick;

    DiagnosticsSnapshot snapshot;
    if (!diagnostics_buffer_.read(snapshot) || !analytical_ik_provider_ ||
        snapshot.status.owner != fr10_motion::MotionOwner::POSE_PATH ||
        !snapshot.status.cartesian_reference_valid ||
        (snapshot.status.state != fr10_motion::MotionState::TRACKING &&
         snapshot.status.state != fr10_motion::MotionState::HOLDING)) {
      path_fault_monitor_.reset();
      return;
    }

    const auto active_policy = active_pose_policy_.load();
    const auto &monitor_config =
        params_.cartesian_fault_monitor_configs[policyIndex(active_policy)];
    if (!path_monitor_policy_initialized_ ||
        monitored_pose_policy_ != active_policy) {
      path_fault_monitor_.setConfig(monitor_config);
      monitored_pose_policy_ = active_policy;
      path_monitor_policy_initialized_ = true;
      RCLCPP_INFO(
          get_logger(),
          "[PATH MONITOR PROFILE] policy=%s along_limit=%.1f mm "
          "persistence=%.1f ms perpendicular_limit=%.1f mm",
          fr10_motion::poseExecutionPolicyName(active_policy),
          monitor_config.tracking_lag_tolerance_m * 1000.0,
          monitor_config.persistence_s * 1000.0,
          monitor_config.perpendicular_tolerance_m * 1000.0);
    }

    fr10_motion::KinematicState joint_state;
    joint_state.position_rad = snapshot.feedback.joint_position_rad;
    joint_state.velocity_rad_s = snapshot.feedback.joint_velocity_rad_s;
    joint_state.acceleration_rad_s2 =
        snapshot.feedback.joint_acceleration_rad_s2;
    fr10_motion::Pose actual_pose;
    std::string error;
    fr10_motion::CartesianTrackingErrors errors;
    if (!analytical_ik_provider_->forward(joint_state, actual_pose, &error)) {
      const double invalid = std::numeric_limits<double>::infinity();
      errors = {invalid, invalid, invalid, invalid};
    } else {
      double squared_position_error = 0.0;
      double along_projection = 0.0;
      double tangent_norm_squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double delta =
            actual_pose.position_m[axis] -
            snapshot.status.desired_cartesian_pose.position_m[axis];
        squared_position_error += delta * delta;
        along_projection +=
            delta * snapshot.status.desired_cartesian_translation_tangent[axis];
        tangent_norm_squared +=
            snapshot.status.desired_cartesian_translation_tangent[axis] *
            snapshot.status.desired_cartesian_translation_tangent[axis];
      }
      errors.position_error_m = std::sqrt(squared_position_error);
      errors.along_track_error_m =
          tangent_norm_squared > 1e-12
              ? std::abs(along_projection) / std::sqrt(tangent_norm_squared)
              : 0.0;
      errors.perpendicular_error_m =
          fr10_motion::CartesianPath::translationDeviation(
              snapshot.status.desired_cartesian_pose,
              snapshot.status.desired_cartesian_translation_tangent,
              actual_pose);
      errors.orientation_error_rad =
          fr10_motion::math::quaternionAngularDistance(
              actual_pose.orientation_quat,
              snapshot.status.desired_cartesian_pose.orientation_quat);
    }

    if (active_policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE) {
      // FREE_SPACE is intentionally joint-space/free-form.  Its sampled
      // Cartesian reference is telemetry, not an invariant path: applying
      // orientation/perpendicular thresholds here silently converts Mode 1
      // back into Cartesian tracking.  Kinematic/collision checks cover every
      // planned sample and the independent joint-reference lead monitor (with
      // the FREE_SPACE persistence window) remains the runtime stop guard.
      path_fault_monitor_.reset();
      motion_executor_->reportCartesianTrackingError(
          errors.position_error_m, errors.orientation_error_rad,
          errors.perpendicular_error_m, false);
      return;
    }

    const auto previous = path_fault_monitor_.state();
    const auto state = path_fault_monitor_.update(
        errors, snapshot.status.state == fr10_motion::MotionState::TRACKING,
        dt);
    const bool persistent =
        state == fr10_motion::CartesianFaultState::PERSISTENT_FAULT;
    // The monitor thread publishes telemetry only. It latches a
    // generation-qualified request for the control runner, which remains the
    // sole writer of executor state.
    motion_executor_->reportCartesianTrackingError(
        errors.position_error_m, errors.orientation_error_rad,
        errors.perpendicular_error_m, false);
    if (state == fr10_motion::CartesianFaultState::WARNING &&
        previous == fr10_motion::CartesianFaultState::NORMAL) {
      RCLCPP_WARN(get_logger(),
                  "[PATH MONITOR WARNING] along=%.6f m perpendicular=%.6f m "
                  "orientation=%.6f rad; persistence threshold=%.3f ms",
                  errors.along_track_error_m, errors.perpendicular_error_m,
                  errors.orientation_error_rad,
                  monitor_config.persistence_s * 1000.0);
    }
    if (persistent &&
        previous != fr10_motion::CartesianFaultState::PERSISTENT_FAULT) {
      RCLCPP_ERROR(get_logger(),
                   "[PATH MONITOR PERSISTENT] duration=%.3f ms position=%.6f m "
                   "along=%.6f m perpendicular=%.6f m orientation=%.6f rad; "
                   "requesting CONTROLLED STOP",
                   path_fault_monitor_.violationDuration() * 1000.0,
                   errors.position_error_m, errors.along_track_error_m,
                   errors.perpendicular_error_m, errors.orientation_error_rad);
      path_stop_generation_.store(snapshot.status.active_generation);
      stop_latch_.store(StopReason::PATH_DEVIATION);
    }
  }

  void diagnosticsTick() {
    DiagnosticsSnapshot snapshot;
    if (!diagnostics_buffer_.read(snapshot)) {
      return;
    }
    auto status = snapshot.status;
    auto feedback = snapshot.feedback;
    fr10_motion::Pose actual_flange;
    const bool actual_flange_valid =
        populateCartesianFeedback(feedback, actual_flange);
    if (actual_flange_valid) {
      fr10_motion::Pose tcp_feedback;
      tcp_feedback.position_m = feedback.tcp_position_m;
      tcp_feedback.orientation_quat = feedback.tcp_orientation_quat;
      tcp_feedback_buffer_.write(tcp_feedback);
      publishTcpPose(feedback);
    }

    double along_track_error_m = 0.0;
    bool tracking = false;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        status.cartesian_reference_valid) {
      double squared_position_error = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double delta = feedback.tcp_position_m[axis] -
                             status.desired_cartesian_pose.position_m[axis];
        squared_position_error += delta * delta;
      }
      status.position_error_m = std::sqrt(squared_position_error);
      status.angular_error_rad = fr10_motion::math::quaternionAngularDistance(
          feedback.tcp_orientation_quat,
          status.desired_cartesian_pose.orientation_quat);
      fr10_motion::Pose actual_pose;
      actual_pose.position_m = feedback.tcp_position_m;
      actual_pose.orientation_quat = feedback.tcp_orientation_quat;
      status.cartesian_path_deviation_m =
          fr10_motion::CartesianPath::translationDeviation(
              status.desired_cartesian_pose,
              status.desired_cartesian_translation_tangent, actual_pose);
      double along_projection = 0.0;
      double tangent_norm_squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double error = feedback.tcp_position_m[axis] -
                             status.desired_cartesian_pose.position_m[axis];
        along_projection +=
            error * status.desired_cartesian_translation_tangent[axis];
        tangent_norm_squared +=
            status.desired_cartesian_translation_tangent[axis] *
            status.desired_cartesian_translation_tangent[axis];
      }
      along_track_error_m =
          tangent_norm_squared > 1e-12
              ? std::abs(along_projection) / std::sqrt(tangent_norm_squared)
              : 0.0;
      tracking = status.state == fr10_motion::MotionState::TRACKING;
    }

    const StopReason event = pending_event_.exchange(StopReason::NONE);
    if (event == StopReason::SINGULARITY) {
      RCLCPP_ERROR(get_logger(),
                   "[TARGET REJECTED: SINGULARITY] Target violates the "
                   "configured kinematic safety barrier.");
    }
    if (event == StopReason::PATH_DEVIATION) {
      const auto &monitor_config = params_.cartesian_fault_monitor_configs[policyIndex(
          active_pose_policy_.load())];
      RCLCPP_ERROR(
          get_logger(),
          "[PATH MONITOR CONTROLLED STOP] state=%s position=%.6f m "
          "along=%.6f m "
          "along_limit=%.6f m orientation=%.6f rad perpendicular=%.6f m "
          "perpendicular_limit=%.6f m",
          tracking ? "TRACKING" : fr10_motion::motionStateName(status.state),
          status.position_error_m, along_track_error_m,
          tracking ? monitor_config.tracking_lag_tolerance_m
                   : params_.cartesian_position_tolerance_m,
          status.angular_error_rad, status.cartesian_path_deviation_m,
          params_.cartesian_path_deviation_tolerance_m);
    }
    if (status.lag_event.sequence > reported_lag_event_) {
      reported_lag_event_ = status.lag_event.sequence;
      const auto &lag = status.lag_event;
      RCLCPP_ERROR(
          get_logger(),
          "[REFERENCE LAG CONTROLLED STOP] event=%lu joint=%zu feedback_seq=%lu "
          "q_cmd=%.6f q_actual=%.6f v_cmd=%.6f age_ms=%.3f "
          "error=%.6f rad limit=%.6f rad",
          lag.sequence, lag.joint + 1, lag.feedback_sequence,
          lag.sent_reference.joint_position_rad[lag.joint],
          lag.feedback.joint_position_rad[lag.joint],
          lag.sent_reference.joint_velocity_rad_s[lag.joint],
          lag.feedback_age_s * 1000.0, lag.error_rad,
          params_.controller_config.max_joint_reference_lead_rad);
    }
    if (event == StopReason::CONTROLLED_STOP) {
      RCLCPP_WARN(get_logger(),
                  "[CONTROLLED STOP] jerk-limited braking profile active");
    }
    if (event == StopReason::RETARGET_REVERSAL) {
      RCLCPP_WARN(get_logger(),
                  "[RETARGET REVERSAL STOP] jerk-limited braking profile "
                  "active; target retained for zero-velocity replan");
    }
    StatusLogSnapshot status_log_snapshot;
    status_log_snapshot.status = status;
    status_log_snapshot.feedback = feedback;
    status_log_snapshot.command = snapshot.command;
    status_log_snapshot.actual_flange = actual_flange;
    status_log_snapshot.loop = snapshot.loop;
    status_log_snapshot.actual_flange_valid = actual_flange_valid;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH) {
      DisplayPoseGoalSnapshot display_goal;
      if (display_pose_goal_buffer_.read(display_goal) && display_goal.valid) {
        status_log_snapshot.final_pose_goal = display_goal.pose;
        status_log_snapshot.final_pose_goal_valid = true;
      } else if (status.cartesian_reference_valid) {
        status_log_snapshot.final_pose_goal = status.cartesian_path_goal;
        status_log_snapshot.final_pose_goal_valid = true;
      }
    }
    status_log_buffer_.write(status_log_snapshot);
    publishStatus(status, feedback, snapshot.command, snapshot.loop);
  }

  void publishStatus(const fr10_motion::MotionStatus &status,
                     const fr10_motion::RobotState &feedback,
                     const fr10_motion::MotionCommand &command,
                     const LoopMetrics &loop) {
    const double mean_rate_hz =
        loop.elapsed_s > 0.0 ? loop.samples / loop.elapsed_s : 0.0;
    const uint64_t sdk_errors = feedback.sdk_read_errors + feedback.sdk_write_errors;
    const auto ik_statistics = analytical_ik_provider_
                                   ? analytical_ik_provider_->statistics()
                                   : fr10_motion::Fr10AnalyticalIKStatistics{};
    double position_error_m = status.position_error_m;
    double angular_error_rad = status.angular_error_rad;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        !status.cartesian_reference_valid) {
      fr10_motion::CartesianTarget target;
      bool have_target = false;
      AcceptedTargetSnapshot accepted_target;
      if (accepted_target_buffer_.read(accepted_target)) {
        have_target = accepted_target.have_cartesian_target;
        target = accepted_target.cartesian_target;
      }
      if (have_target) {
        double squared_error = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double delta =
              target.position_m[axis] - feedback.tcp_position_m[axis];
          squared_error += delta * delta;
        }
        position_error_m = std::sqrt(squared_error);
        angular_error_rad = fr10_motion::math::quaternionAngularDistance(
            target.orientation_quat, feedback.tcp_orientation_quat);
      }
    }

    std::array<char, 1536> buffer{};
    const int length = std::snprintf(
        buffer.data(), buffer.size(),
        "state=%s outcome=%s mode=%s pose_policy=%s owner=%s type=%s "
        "revision=%lu generation=%lu scene_revision=%lu "
        "active_scene_revision=%lu command_seq=%lu feedback_seq=%lu "
        "feedback_age_ms=%.3f safety_known=%s "
        "backend_ready=%s loop_samples=%lu loop_hz=%.3f "
        "mean_cycle_period_ms=%.6f "
        "mean_abs_jitter_ms=%.6f max_abs_jitter_ms=%.6f "
        "max_cycle_period_ms=%.6f "
        "ik_mean_us=%.3f ik_max_us=%.3f "
        "position_error_m=%.6f angular_error_rad=%.6f path_deviation_m=%.6f "
        "reference_actual_rad=%.6f candidate_reference_actual_rad=%.6f "
        "reference_lagging=%s "
        "reference_lag_samples=%u "
        "max_delta_qdot_rad_s=%.6f max_delta_v_m_s=%.6f "
        "sdk_read_errors=%lu sdk_write_errors=%lu command_write_errors=%lu "
        "sdk_errors=%lu "
        "reason=%s",
        fr10_motion::motionStateName(status.state),
        fr10_motion::motionOutcomeName(status.outcome),
        fr10_motion::controlModeName(active_control_mode_.load()),
        fr10_motion::poseExecutionPolicyName(active_pose_policy_.load()),
        fr10_motion::motionOwnerName(status.owner),
        fr10_motion::commandTypeName(status.active_type),
        status.active_revision, status.active_generation,
        scene_revision_.load(std::memory_order_acquire),
        active_scene_revision_.load(std::memory_order_acquire),
        command.sequence,
        feedback.feedback_sequence, feedback.feedback_age_s * 1000.0,
        feedback.safety_known ? "true" : "false",
        backend_ready_announced_.load() ? "true" : "false", loop.samples,
        mean_rate_hz,
        loop.samples > 0
            ? (loop.elapsed_s / static_cast<double>(loop.samples)) * 1000.0
            : 0.0,
        loop.mean_abs_jitter_ms, loop.max_abs_jitter_ms,
        loop.max_cycle_period_ms, ik_statistics.mean_solve_time_us,
        ik_statistics.max_solve_time_us, position_error_m, angular_error_rad,
        status.cartesian_path_deviation_m,
        status.max_reference_actual_error_rad,
        status.candidate_reference_error_rad,
        status.reference_lagging ? "true" : "false",
        status.reference_lag_samples,
        loop.max_delta_joint_velocity_rad_s, loop.max_delta_linear_velocity_m_s,
        feedback.sdk_read_errors, feedback.sdk_write_errors,
        loop.command_write_errors, sdk_errors,
        stopReasonName(stop_reason_.load()));
    if (length > 0) {
      status_message_.data.assign(
          buffer.data(), static_cast<std::size_t>(std::min<int>(
                             length, static_cast<int>(buffer.size() - 1))));
      status_pub_->publish(status_message_);
      tracking_diagnostics_pub_->publish(status_message_);
    }

    // Goal Reached event: 1-time log when transitioning from TRACKING ->
    // HOLDING
    if (previous_reported_state_ == fr10_motion::MotionState::STOPPING &&
        status.state == fr10_motion::MotionState::HOLDING) {
      RCLCPP_INFO(get_logger(),
                  "[CONTROLLED STOP COMPLETE] qdot reference reached zero; "
                  "robot command is HOLDING");
    } else if (status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED &&
               previous_reported_state_ != fr10_motion::MotionState::HOLDING &&
               status.state == fr10_motion::MotionState::HOLDING) {
      if (status.owner == fr10_motion::MotionOwner::POSE_PATH) {
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Position error: %.1f mm / angular "
                    "error: %.1f deg. Stable HOLD.",
                    position_error_m * 1000.0,
                    angular_error_rad * fr10_motion::kRadToDeg);
      } else if (status.active_type ==
                 fr10_motion::CommandType::JOINT_POSITION) {
        double max_joint_err_deg = 0.0;
        for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
          max_joint_err_deg =
              std::max(max_joint_err_deg, std::abs(status.joint_error_rad[i]) *
                                              fr10_motion::kRadToDeg);
        }
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Maximum joint error: %.2f deg. "
                    "Stable HOLD.",
                    max_joint_err_deg);
      } else {
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Robot is in stable HOLD.");
      }
    }

    previous_reported_state_ = status.state;
  }

  void statusLogTick() {
    StatusLogSnapshot snapshot;
    if (!status_log_buffer_.read(snapshot) || !snapshot.actual_flange_valid) {
      return;
    }

    std::array<double, fr10_motion::kJointCount> target_joint_rad =
        snapshot.feedback.joint_position_rad;
    if (snapshot.status.owner == fr10_motion::MotionOwner::JOINT_OTG) {
      AcceptedTargetSnapshot accepted_target;
      if (accepted_target_buffer_.read(accepted_target) &&
          accepted_target.have_joint_target) {
        target_joint_rad = accepted_target.joint_target.position_rad;
      }
    } else if (snapshot.command.type ==
               fr10_motion::CommandType::JOINT_POSITION) {
      target_joint_rad = snapshot.command.joint_position_rad;
    }

    fr10_motion::Pose target_tcp;
    if (snapshot.status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        snapshot.final_pose_goal_valid) {
      target_tcp = snapshot.final_pose_goal;
    } else {
      fr10_motion::KinematicState target_state;
      target_state.position_rad = target_joint_rad;
      std::string fk_error;
      if (!moveit_ik_provider_->forward(target_state, target_tcp, &fk_error)) {
        RCLCPP_WARN(get_logger(), "Unified diagnostic target FK failed: %s",
                    fk_error.c_str());
        return;
      }
    }

    const auto diagnostic =
        fr10_motion::diagnostics::makeUnifiedTrackingDiagnostic(
            target_tcp, snapshot.actual_flange, flange_to_tcp_, target_joint_rad,
            snapshot.feedback.joint_position_rad);
    const double mean_rate_hz =
        snapshot.loop.elapsed_s > 0.0
            ? snapshot.loop.samples / snapshot.loop.elapsed_s
            : params_.controller_config.control_rate_hz;
    const char *mode =
        snapshot.status.owner == fr10_motion::MotionOwner::POSE_PATH
            ? "POSE"
            : (snapshot.command.type == fr10_motion::CommandType::JOINT_POSITION
                   ? "JOINT"
                   : "IDLE");
    const std::string block =
        fr10_motion::diagnostics::formatUnifiedTrackingBlock(
            fr10_motion::motionStateName(snapshot.status.state), mode,
            mean_rate_hz, diagnostic);
    if (block.empty()) {
      return;
    }
    const std::string log_key = fr10_motion::diagnostics::unifiedTrackingLogKey(
        fr10_motion::motionStateName(snapshot.status.state), mode, diagnostic);
    if (!fr10_motion::diagnostics::shouldEmitChangedTrackingLog(
            log_key, last_status_log_key_)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", block.c_str());
  }

  fr10_pose_controller::NodeParameters params_;
  std::unique_ptr<fr10_motion::MotionExecutor> motion_executor_;
  std::unique_ptr<fr10_motion::RobotBackend> backend_;
  std::atomic<bool> backend_shutdown_{false};
  std::atomic<bool> backend_ready_announced_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_in_progress_{false};
  std::atomic<StopReason> stop_reason_{StopReason::NONE};
  std::atomic<StopReason> pending_event_{StopReason::NONE};
  std::atomic<StopReason> stop_latch_{StopReason::NONE};
  std::atomic<uint64_t> path_stop_generation_{0};
  uint64_t handled_lag_event_{0};
  uint64_t reported_lag_event_{0};
  std::atomic<fr10_motion::ControlMode> active_control_mode_{
      fr10_motion::ControlMode::JOINT};
  std::atomic<fr10_motion::PoseExecutionPolicy> active_pose_policy_{
      fr10_motion::PoseExecutionPolicy::FREE_SPACE};
  std::atomic<uint64_t> scene_revision_{1};
  std::atomic<uint64_t> active_scene_revision_{0};

  std::string last_status_log_key_;
  fr10_motion::CartesianFaultMonitor path_fault_monitor_;
  fr10_motion::PoseExecutionPolicy monitored_pose_policy_{
      fr10_motion::PoseExecutionPolicy::FREE_SPACE};
  bool path_monitor_policy_initialized_{false};

  fr10_motion::RealtimeBuffer<fr10_motion::RobotState> feedback_buffer_;
  fr10_motion::RealtimeBuffer<fr10_motion::Pose> tcp_feedback_buffer_;
  fr10_motion::RealtimeBuffer<fr10_motion::MotionCommand>
      latest_command_buffer_;
  fr10_motion::Pose flange_to_tcp_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup *joint_model_group_{nullptr};
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<fr10_pose_controller::MoveItIKProvider> moveit_ik_provider_;
  std::unique_ptr<fr10_motion::Fr10AnalyticalIKProvider>
      analytical_ik_provider_;
  std::unique_ptr<fr10_pose_controller::CollisionValidatedIKProvider>
      collision_validated_ik_provider_;
  fr10_motion::IKProvider *path_ik_provider_{nullptr};

  std::array<std::size_t, fr10_motion::kJointCount> joint_command_index_map_{};
  std::vector<std::string> joint_command_layout_;
  bool joint_index_map_valid_{false};

  std::mutex kinematics_mutex_;
  std::condition_variable kinematics_cv_;
  std::optional<KinematicsTask> pending_kinematics_task_;
  std::mutex reversal_mutex_;
  std::optional<KinematicsTask> post_stop_replan_task_;
  std::mutex insertion_sequence_mutex_;
  std::optional<PendingInsertionSequence> pending_insertion_sequence_;
  std::optional<fr10_motion::CartesianTarget> last_insertion_target_;
  std::thread kinematics_worker_;
  // Worker-owned retirement: large trajectory destruction never occurs on
  // the tick that replaces or aborts a plan.
  std::vector<std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory>> retained_trajectories_;
  std::atomic<uint64_t> kinematics_task_sequence_{0};

  fr10_motion::RealtimeBuffer<KinematicsResult> kinematics_result_buffer_;
  uint64_t consumed_kinematics_result_sequence_{0};
  AcceptedTargetSnapshot accepted_target_writer_;
  fr10_motion::RealtimeBuffer<AcceptedTargetSnapshot> accepted_target_buffer_;
  fr10_motion::RealtimeBuffer<DisplayPoseGoalSnapshot>
      display_pose_goal_buffer_;

  std::mutex target_mutex_;
  fr10_motion::CartesianTarget last_queued_cartesian_target_;
  bool have_queued_cartesian_target_{false};
  std::atomic<uint64_t> joint_revision_{0};
  std::atomic<uint64_t> cartesian_generation_{0};

  fr10_motion::RealtimeBuffer<DiagnosticsSnapshot> diagnostics_buffer_;
  fr10_motion::RealtimeBuffer<StatusLogSnapshot> status_log_buffer_;

  fr10_motion::MotionState previous_reported_state_{
      fr10_motion::MotionState::IDLE};

  int consecutive_feedback_failures_{0};
  uint64_t command_write_errors_{0};
  fr10_motion::MotionCommand previous_command_;
  bool have_previous_command_{false};
  double max_delta_joint_velocity_rad_s_{0.0};
  double max_delta_linear_velocity_m_s_{0.0};
  uint64_t loop_samples_{0};
  double loop_elapsed_s_{0.0};
  double loop_abs_jitter_sum_ms_{0.0};
  double max_abs_jitter_ms_{0.0};
  double max_cycle_period_ms_{0.0};
  std::chrono::steady_clock::time_point last_tick_{};

  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr target_callback_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  rclcpp::CallbackGroup::SharedPtr path_monitor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      tracking_diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr retract_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr controlled_stop_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr path_monitor_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr status_log_timer_;
  geometry_msgs::msg::PoseStamped tcp_pose_message_;
  std_msgs::msg::String status_message_;
  std::chrono::steady_clock::time_point path_monitor_last_tick_{};
};

} // namespace

int fr10_pose_controller::runMotionAdapter(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);

  int stop_pipe[2] = {-1, -1};
  if (::pipe2(stop_pipe, O_CLOEXEC | O_NONBLOCK) != 0) {
    RCLCPP_FATAL(rclcpp::get_logger("fr10_pose_controller"),
                 "Failed to create signal wakeup pipe");
    rclcpp::shutdown();
    return 1;
  }
  stop_pipe_write_fd = stop_pipe[1];

  const auto previous_sigint = std::signal(SIGINT, signalHandler);
  const auto previous_sigterm = std::signal(SIGTERM, signalHandler);

  int return_code = 0;
  bool backend_clean = true;
  std::shared_ptr<MotionAdapter> controller;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    6);
  std::thread spin_thread;

  try {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    controller = std::make_shared<MotionAdapter>(options);
    controller->initialize();
    executor.add_node(controller);
    spin_thread = std::thread([&executor]() { executor.spin(); });

    pollfd signal_poll{stop_pipe[0], POLLIN, 0};
    while (rclcpp::ok() && stop_signal == 0 && !controller->stopRequested()) {
      const int poll_result = ::poll(&signal_poll, 1, 5);
      if (poll_result > 0 && (signal_poll.revents & POLLIN) != 0) {
        std::array<uint8_t, 16> signal_bytes{};
        const ssize_t read_result =
            ::read(stop_pipe[0], signal_bytes.data(), signal_bytes.size());
        (void)read_result;
      }
    }
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("fr10_pose_controller"), "%s",
                 exception.what());
    return_code = 1;
  }

  // 1. Keep the executor spinning until the braking stream reaches HOLDING.
  if (controller) {
    controller->beginGracefulShutdown();
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(controller->shutdownTimeoutS());
    while (!controller->stopRequested() && rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!controller->stopRequested()) {
      RCLCPP_ERROR(controller->get_logger(), "Shutdown braking deadline expired; completion unconfirmed");
      return_code = 1;
    }
    controller->requestStopAll(StopReason::USER_INTERRUPT);
  }

  // 2. No backend call can race after the executor has stopped and joined.
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  // 3. Managed workers join only after executor callbacks have stopped.
  if (controller) {
    controller->cleanupWorkers();
  }

  // 4. Detach the node before destroying its callback groups and timers.
  if (controller) {
    try {
      executor.remove_node(controller);
    } catch (...) {
      return_code = 1;
    }
  }

  // 5. Release the sole hardware owner, then the node.
  if (controller) {
    backend_clean = controller->shutdownBackend();
  }
  controller.reset();

  stop_pipe_write_fd = -1;
  ::close(stop_pipe[0]);
  ::close(stop_pipe[1]);

  // 6. The ROS context is the final resource released.
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  std::signal(SIGINT, previous_sigint);
  std::signal(SIGTERM, previous_sigterm);

  if (!backend_clean) {
    return_code = 1;
  }
  return return_code;
}
