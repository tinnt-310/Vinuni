#include "motion_adapter_internal.hpp"

namespace fr10_pose_controller::detail {

MotionAdapter::MotionAdapter(const rclcpp::NodeOptions &options)
      : Node("fr10_pose_controller", options),
        params_(fr10_pose_controller::NodeParameters::declare(this)) {
    force_runtime_config_ =
        fr10_pose_controller::declareForceRuntimeConfig(
            this, params_.kinematics_tip_link);
    const auto force_model_description = force_runtime_config_.enabled
        ? (has_parameter("robot_description") ? get_parameter("robot_description").as_string()
           : declare_parameter<std::string>("robot_description", ""))
        : std::string{};
    const bool verified_native_simulator = force_runtime_config_.native_simulator_verified &&
        force_runtime_config_.native_simulator_endpoint == "192.168.58.2" &&
        force_runtime_config_.native_simulator_image_digest ==
            "sha256:7f4124d7fcfa95a604802bff20c1ae0f65a18b5d2e782cb88f49921dc63daab2" &&
        force_runtime_config_.profile_id == "matched_socket_v2" &&
        force_model_description.find("<param name=\"robot_ip\">192.168.58.2</param>") != std::string::npos;
    if (force_runtime_config_.enabled &&
        (force_model_description.empty() ||
         (force_model_description.find("FairinoHardwareInterface") != std::string::npos &&
          !verified_native_simulator))) {
      throw std::runtime_error(
          "Force deployment on FAIRINO is locked: real sensor commissioning and a verified "
          "compatible SimMachine transport gate are not complete. Simulation profiles cannot enable real contact.");
    }
    if (force_runtime_config_.session.state_machine.process_mode ==
        fr10_insertion::InsertionProcessMode::AXIAL_CONTACT) {
      force_runtime_config_.wrench_adapter.lateral_tolerance_m =
          params_.linear_insert_config.perpendicular_tolerance_m;
      force_runtime_config_.wrench_adapter.orientation_tolerance_rad =
          params_.linear_insert_config.orientation_tolerance_rad;
    }
    path_fault_monitor_.setConfig(params_.cartesian_fault_monitor_configs[
        policyIndex(fr10_motion::PoseExecutionPolicy::FREE_SPACE)]);
    motion_executor_ =
        std::make_unique<fr10_motion::MotionExecutor>(params_.controller_config);
    coordinator_ = std::make_unique<fr10_motion::MotionCoordinator>(
        params_.joint_change_tolerance_rad, params_.pose_position_change_tolerance_m,
        params_.pose_angular_change_tolerance_rad);

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
    if (force_runtime_config_.enabled) {
      force_joint_trace_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/fr10_force/joint_reference", rclcpp::SensorDataQoS());
      force_joint_trace_message_.data.resize(18);
      force_requires_base_feedback_ = force_model_description.find("Wheel1_to_Revolute1") != std::string::npos;
      const auto base_topic = declare_parameter<std::string>("force.base_motion_topic", "/odom");
      const auto base_frame = declare_parameter<std::string>("force.base_motion_frame", "odom");
      const auto twist_source = declare_parameter<std::string>("force.base_motion_twist_source", "message");
      const auto pose_window_s = declare_parameter<double>("force.base_motion_pose_window_s", .016);
      if (!std::isfinite(pose_window_s) || pose_window_s < .001 || pose_window_s > .020)
        throw std::invalid_argument("force.base_motion_pose_window_s must be within 1–20 ms");
      if(twist_source!="message" && twist_source!="pose")
        throw std::invalid_argument("force.base_motion_twist_source must be message or pose");
      if (force_requires_base_feedback_)
        force_parking_ = std::make_unique<fr10_insertion::ParkingRosAdapter>(this, base_topic, base_frame, twist_source=="pose", pose_window_s);
    }
    // Parameters are startup configuration; never accept changes that the
    // live executor would silently ignore.
    parameter_guard_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters) {
            if (parameter.get_name().rfind("controller.", 0) == 0 ||
                parameter.get_name().rfind("force.", 0) == 0) {
              result.successful = false;
              result.reason = "controller parameters require restart";
              break;
            }
          }
          return result;
        });

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/controller_status", rclcpp::QoS(1).reliable().transient_local());
    command_status_pub_ = create_publisher<fr10_motion_interfaces::msg::MotionCommandStatus>(
        "/fr10_motion/command_status", rclcpp::QoS(64).reliable().transient_local());
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
    free_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fr10_motion/pose_target", rclcpp::QoS(10).reliable(),
        std::bind(&MotionAdapter::freePoseTargetCallback, this,
                  std::placeholders::_1), target_subscription_options);
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
    force_adapter_ =
        std::make_unique<fr10_insertion::InsertionRosAdapter>(
            this, force_runtime_config_.session, params_.base_frame,
            force_runtime_config_.wrench_adapter,
            force_runtime_config_.profile_id, force_runtime_config_.enabled,
            [this]() { return forceGoalCanStart(); });

    tcp_pose_message_.header.frame_id = params_.base_frame;
    status_message_.data.reserve(1536);
  }

MotionAdapter::~MotionAdapter() {
    requestStopAll();
    shutdownBackend();
  }

void MotionAdapter::initialize() {
    initializeKinematics();

    if (params_.backend_name != "ros_control") {
      throw std::runtime_error(
          "Pose/Joint runtime requires controller.backend=ros_control");
    }
    backend_ = std::make_unique<fr10_motion::RosControlBackend>(
        shared_from_this(), params_.joint_names, params_.command_topic, params_.joint_state_topic,
        feedback_callback_group_, params_.feedback_timeout_s,
        params_.require_driver_health);
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
    RCLCPP_INFO(get_logger(),
                "Force insertion action is %s (profile=%s, wrench=%s -> %s)",
                force_runtime_config_.enabled ? "ENABLED" : "DISABLED",
                force_runtime_config_.profile_id.c_str(),
                force_runtime_config_.wrench_adapter.source_frame.c_str(),
                force_runtime_config_.wrench_adapter.target_frame.c_str());
    DiagnosticsSnapshot initial_snapshot;
    initial_snapshot.status = motion_executor_->status();
    initial_snapshot.feedback = initial_state;
    initial_snapshot.command = motion_executor_->command();
    diagnostics_buffer_.write(initial_snapshot);
    diagnosticsTick();
    statusLogTick();
  }

} // namespace fr10_pose_controller::detail

namespace {
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

} // namespace

using fr10_pose_controller::detail::MotionAdapter;
using fr10_motion::StopReason;

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
