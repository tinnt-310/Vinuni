#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_core/continuous_ik.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "test_support.hpp"

namespace {

class LinearPoseIK final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult
  solve(const fr10_motion::Pose &target,
        const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    result.success = true;
    result.state.position_rad[0] = target.position_m[0];
    result.state.position_rad[1] = target.position_m[1];
    result.state.position_rad[2] = target.position_m[2];
    result.state.position_rad[3] =
        fr10_motion::math::quaternionToRotationVector(
            target.orientation_quat)[2];
    result.manipulability = 1.0;
    result.condition_number = 2.0;
    return result;
  }

  fr10_motion::IKResult
  validate(const fr10_motion::KinematicState &target,
           const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    result.success = true;
    result.state = target;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string * /* error */) const override {
    pose.position_m = {state.position_rad[0], state.position_rad[1],
                       state.position_rad[2]};
    pose.orientation_quat =
        fr10_motion_test::axisAngle(2, state.position_rad[3]);
    return true;
  }
};

struct Metrics {
  bool success{false};
  double max_perpendicular_error_m{0.0};
  double rms_position_error_m{0.0};
  double max_orientation_error_rad{0.0};
  fr10_motion::PathTimeParameterizationResult timing;
};

bool validTimeLaw(const Metrics &metrics) {
  if (!metrics.success || metrics.timing.trajectory.points.size() < 2) {
    return false;
  }
  const auto &points = metrics.timing.trajectory.points;
  if (std::abs(points.front().time_from_start_s) > 1e-12 ||
      std::abs(points.front().path_parameter) > 1e-12 ||
      std::abs(points.back().path_parameter - 1.0) > 1e-12 ||
      metrics.timing.duration_s + 1e-12 < 0.40 ||
      metrics.timing.max_joint_jerk_rad_s3 > 15.0 + 1e-7) {
    return false;
  }
  for (std::size_t index = 1; index < points.size(); ++index) {
    if (!std::isfinite(points[index].time_from_start_s) ||
        !std::isfinite(points[index].path_parameter) ||
        points[index].time_from_start_s <=
            points[index - 1].time_from_start_s ||
        points[index].path_parameter + 1e-12 <
            points[index - 1].path_parameter) {
      return false;
    }
  }
  return true;
}

Metrics runCase(
    const fr10_motion::Pose &start, const fr10_motion::Pose &goal,
    uint64_t generation,
    const fr10_motion::KinematicState *initial_override = nullptr,
    const fr10_motion::PathTimeParameterizerConfig *timing_override = nullptr) {
  Metrics metrics;
  fr10_motion::CartesianPathConfig path_config;
  path_config.max_translation_step_m = 0.002;
  path_config.max_rotation_step_rad = 0.01;
  std::vector<fr10_motion::Pose> cartesian_path;
  std::string error;
  if (!fr10_motion::CartesianPath::linear(start, goal, path_config,
                                          cartesian_path, &error)) {
    return metrics;
  }

  LinearPoseIK provider;
  fr10_motion::KinematicState seed = provider.solve(start, {}).state;
  if (initial_override != nullptr) {
    seed.velocity_rad_s = initial_override->velocity_rad_s;
    seed.acceleration_rad_s2 = initial_override->acceleration_rad_s2;
  }
  fr10_motion::ContinuousIKConfig ik_config;
  ik_config.max_joint_step_rad = 0.05;
  ik_config.max_position_error_m = 1e-10;
  ik_config.max_orientation_error_rad = 1e-10;
  const auto ik = fr10_motion::ContinuousIK::solve(cartesian_path, seed,
                                                   provider, ik_config);
  if (!ik.success) {
    return metrics;
  }

  fr10_motion::PathTimeParameterizerConfig timing_config;
  timing_config.sample_period_s = 0.008;
  timing_config.limits = fr10_motion_test::limits();
  timing_config.joint_velocity_scale = 0.8;
  timing_config.joint_acceleration_scale = std::sqrt(0.8);
  timing_config.joint_jerk_scale = 0.8;
  timing_config.linear_scale = 0.5;
  timing_config.angular_scale = 0.5;
  if (timing_override != nullptr) {
    timing_config = *timing_override;
  }
  metrics.timing = fr10_motion::PathTimeParameterizer::parameterize(
      cartesian_path, ik.joint_path, seed, timing_config, generation);
  if (!metrics.timing.success) {
    return metrics;
  }

  double squared_error_sum = 0.0;
  for (const auto &point : metrics.timing.trajectory.points) {
    fr10_motion::Pose actual;
    if (!provider.forward(point.joint_state, actual, nullptr)) {
      return metrics;
    }
    double squared_error = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          actual.position_m[axis] - point.desired_pose.position_m[axis];
      squared_error += delta * delta;
    }
    squared_error_sum += squared_error;
    metrics.max_perpendicular_error_m = std::max(
        metrics.max_perpendicular_error_m,
        fr10_motion::CartesianPath::translationDeviation(start, goal, actual));
    metrics.max_orientation_error_rad = std::max(
        metrics.max_orientation_error_rad,
        fr10_motion::math::quaternionAngularDistance(
            actual.orientation_quat, point.desired_pose.orientation_quat));
  }
  metrics.rms_position_error_m =
      std::sqrt(squared_error_sum /
                static_cast<double>(metrics.timing.trajectory.points.size()));
  metrics.success = true;
  return metrics;
}

bool limitsSatisfied(const Metrics &metrics) {
  const auto limits = fr10_motion_test::limits();
  return metrics.timing.max_joint_velocity_rad_s <= 0.8 * 0.30 + 1e-7 &&
         metrics.timing.max_joint_acceleration_rad_s2 <=
             std::sqrt(0.8) * 0.80 + 1e-7 &&
         metrics.timing.max_joint_jerk_rad_s3 <= 0.8 * 4.0 + 1e-7 &&
         metrics.timing.max_linear_velocity_m_s <= 0.5 * 0.20 + 1e-7 &&
         metrics.timing.max_linear_acceleration_m_s2 <=
             std::sqrt(0.5) * 0.80 + 1e-7 &&
         metrics.timing.max_linear_jerk_m_s3 <= 0.5 * 4.0 + 1e-7 &&
         metrics.timing.max_angular_velocity_rad_s <=
             0.5 * limits.max_angular_velocity_rad_s + 1e-7 &&
         metrics.timing.max_angular_acceleration_rad_s2 <=
             std::sqrt(0.5) * limits.max_angular_acceleration_rad_s2 + 1e-7 &&
         metrics.timing.max_angular_jerk_rad_s3 <=
             0.5 * limits.max_angular_jerk_rad_s3 + 1e-7;
}

} // namespace

int main() {
  fr10_motion_test::Check check;
  fr10_motion::Pose start;
  start.position_m = {0.20, -0.10, 0.40};

  uint64_t generation = 0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    auto goal = start;
    goal.position_m[axis] += 0.050;
    const Metrics metrics = runCase(start, goal, ++generation);
    const std::string id = "PATH_TIME_BASE_AXIS_" + std::to_string(axis);
    check.require(metrics.success && validTimeLaw(metrics) &&
                      limitsSatisfied(metrics) &&
                      metrics.max_perpendicular_error_m < 1e-10 &&
                      metrics.rms_position_error_m < 1e-10 &&
                      metrics.max_orientation_error_rad < 1e-10,
                  id, metrics.timing.message);
    std::cout
        << "METRIC " << id
        << " max_perpendicular_error_m=" << metrics.max_perpendicular_error_m
        << " rms_position_error_m=" << metrics.rms_position_error_m
        << " max_orientation_error_rad=" << metrics.max_orientation_error_rad
        << " duration_s=" << metrics.timing.duration_s
        << " max_joint_velocity_rad_s="
        << metrics.timing.max_joint_velocity_rad_s
        << " max_joint_acceleration_rad_s2="
        << metrics.timing.max_joint_acceleration_rad_s2
        << " max_joint_jerk_rad_s3=" << metrics.timing.max_joint_jerk_rad_s3
        << " max_linear_velocity_m_s=" << metrics.timing.max_linear_velocity_m_s
        << " s_start="
        << metrics.timing.trajectory.points.front().path_parameter
        << " s_end=" << metrics.timing.trajectory.points.back().path_parameter
        << "\n";
  }

  auto se3_goal = start;
  se3_goal.position_m[0] += 0.030;
  se3_goal.orientation_quat = fr10_motion_test::axisAngle(2, 0.20);
  const Metrics se3 = runCase(start, se3_goal, ++generation);
  check.require(se3.success && validTimeLaw(se3) && limitsSatisfied(se3) &&
                    se3.max_perpendicular_error_m < 1e-10 &&
                    se3.rms_position_error_m < 1e-10 &&
                    se3.max_orientation_error_rad < 1e-7 &&
                    se3.timing.max_angular_velocity_rad_s > 0.0,
                "PATH_TIME_TRANSLATION_SLERP", se3.timing.message);
  check.require(
      std::abs(se3.timing.diagnostics.trajectory_distance_m - 0.030) < 1e-9 &&
          std::abs(se3.timing.diagnostics.trajectory_rotation_rad - 0.20) <
              1e-9 &&
          se3.timing.diagnostics.duration_s == se3.timing.duration_s &&
          se3.timing.diagnostics.limiting_constraint !=
              fr10_motion::TimeParameterConstraint::NONE &&
          se3.timing.diagnostics.scaling_iterations > 0,
      "PATH_TIME_DIAGNOSTIC_TELEMETRY",
      fr10_motion::timeParameterConstraintName(
          se3.timing.diagnostics.limiting_constraint));
  std::cout << "METRIC PATH_TIME_TRANSLATION_SLERP"
            << " max_perpendicular_error_m=" << se3.max_perpendicular_error_m
            << " rms_position_error_m=" << se3.rms_position_error_m
            << " max_orientation_error_rad=" << se3.max_orientation_error_rad
            << " max_angular_velocity_rad_s="
            << se3.timing.max_angular_velocity_rad_s
            << " max_angular_acceleration_rad_s2="
            << se3.timing.max_angular_acceleration_rad_s2
            << " max_angular_jerk_rad_s3=" << se3.timing.max_angular_jerk_rad_s3
            << "\n";

  const auto policyConfig =
      [](fr10_motion::PoseExecutionPolicy policy, double joint_velocity,
         double joint_acceleration, double joint_jerk, double linear,
         double angular) {
    fr10_motion::PathTimeParameterizerConfig config;
    config.sample_period_s = 0.008;
    config.limits = fr10_motion_test::limits();
    config.policy = policy;
    config.joint_velocity_scale = joint_velocity;
    config.joint_acceleration_scale = joint_acceleration;
    config.joint_jerk_scale = joint_jerk;
    config.linear_scale = linear;
    config.angular_scale = angular;
    return config;
  };
  auto long_goal = start;
  long_goal.position_m[0] += 0.420;
  const auto free_config = policyConfig(
      fr10_motion::PoseExecutionPolicy::FREE_SPACE, 1.0, 1.0, 1.0, 1.0,
      0.75);
  const auto insertion_config = policyConfig(
      fr10_motion::PoseExecutionPolicy::INSERTION, 0.25, 0.50, 0.25, 0.075,
      0.25);
  const Metrics free_mode =
      runCase(start, long_goal, ++generation, nullptr, &free_config);
  const Metrics insertion_mode =
      runCase(start, long_goal, ++generation, nullptr, &insertion_config);
  check.require(
      free_mode.success && insertion_mode.success &&
          free_mode.timing.max_linear_velocity_m_s > 0.19 &&
          insertion_mode.timing.max_linear_velocity_m_s > 0.014 &&
          insertion_mode.timing.max_linear_velocity_m_s <= 0.015 + 1e-7 &&
          free_mode.timing.duration_s < insertion_mode.timing.duration_s,
      "PATH_TIME_TWO_POSE_POLICIES", insertion_mode.timing.message);
  std::cout << "METRIC PATH_TIME_TWO_POSE_POLICIES"
            << " free_duration_s=" << free_mode.timing.duration_s
            << " insertion_duration_s=" << insertion_mode.timing.duration_s
            << " free_max_linear_m_s="
            << free_mode.timing.max_linear_velocity_m_s
            << " insertion_max_linear_m_s="
            << insertion_mode.timing.max_linear_velocity_m_s << "\n";

  fr10_motion::Pose detour_goal = start;
  detour_goal.position_m[0] += 0.060;
  std::vector<fr10_motion::Pose> detour_path;
  fr10_motion::CartesianPathConfig detour_path_config;
  detour_path_config.max_translation_step_m = 0.002;
  detour_path_config.max_rotation_step_rad = 0.01;
  std::string detour_error;
  check.require(fr10_motion::CartesianPath::detour(
                    start, detour_goal, 0.5, {0.0, 0.040, 0.0},
                    detour_path_config, detour_path, &detour_error),
                "PATH_TIME_DETOUR_CREATE", detour_error);
  LinearPoseIK detour_provider;
  fr10_motion::KinematicState detour_seed =
      detour_provider.solve(start, {}).state;
  fr10_motion::ContinuousIKConfig detour_ik_config;
  detour_ik_config.max_joint_step_rad = 0.05;
  detour_ik_config.max_position_error_m = 1e-10;
  detour_ik_config.max_orientation_error_rad = 1e-10;
  const auto detour_ik = fr10_motion::ContinuousIK::solve(
      detour_path, detour_seed, detour_provider, detour_ik_config);
  fr10_motion::PathTimeParameterizerConfig detour_timing_config;
  detour_timing_config.sample_period_s = 0.008;
  detour_timing_config.limits = fr10_motion_test::limits();
  detour_timing_config.joint_velocity_scale = 0.8;
  detour_timing_config.joint_acceleration_scale = std::sqrt(0.8);
  detour_timing_config.joint_jerk_scale = 0.8;
  detour_timing_config.linear_scale = 0.5;
  detour_timing_config.angular_scale = 0.5;
  const auto detour_timing =
      detour_ik.success ? fr10_motion::PathTimeParameterizer::parameterize(
                              detour_path, detour_ik.joint_path, detour_seed,
                              detour_timing_config, ++generation)
                        : fr10_motion::PathTimeParameterizationResult{};
  double detour_max_error = 0.0;
  double detour_max_y = 0.0;
  for (const auto &point : detour_timing.trajectory.points) {
    fr10_motion::Pose actual;
    detour_provider.forward(point.joint_state, actual, nullptr);
    double squared_error = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          actual.position_m[axis] - point.desired_pose.position_m[axis];
      squared_error += delta * delta;
    }
    detour_max_error = std::max(detour_max_error, std::sqrt(squared_error));
    detour_max_y = std::max(detour_max_y, point.desired_pose.position_m[1] -
                                              start.position_m[1]);
  }
  check.require(detour_ik.success && detour_timing.success &&
                    detour_max_error < 1e-4 && detour_max_y > 0.035,
                "PATH_TIME_CURVED_ROUTE",
                detour_timing.message.empty() ? detour_ik.message
                                              : detour_timing.message);

  fr10_motion::Pose socket = start;
  socket.orientation_quat =
      fr10_motion_test::axisAngle(2, 0.25 * std::acos(-1.0));
  for (std::size_t local_axis = 0; local_axis < 3; ++local_axis) {
    std::array<double, 3> axis{};
    axis[local_axis] = 1.0;
    for (const double distance_m : {0.010, 0.020, 0.030, 0.050, 0.070}) {
      fr10_motion::Pose pre_insert;
      std::string error;
      const bool created = fr10_motion::CartesianPath::preInsertPose(
          socket, axis, distance_m, pre_insert, &error);
      double measured_distance_squared = 0.0;
      for (std::size_t base_axis = 0; base_axis < 3; ++base_axis) {
        const double delta =
            socket.position_m[base_axis] - pre_insert.position_m[base_axis];
        measured_distance_squared += delta * delta;
      }
      check.require(
          created &&
              std::abs(std::sqrt(measured_distance_squared) - distance_m) <
                  1e-12 &&
              fr10_motion::math::quaternionAngularDistance(
                  socket.orientation_quat, pre_insert.orientation_quat) < 1e-12,
          "PRE_INSERT_AXIS_DISTANCE_" + std::to_string(local_axis) + "_" +
              std::to_string(static_cast<int>(distance_m * 1000.0)),
          error);
    }
  }

  fr10_motion::Pose pre_insert;
  std::string pre_insert_error;
  check.require(
      fr10_motion::CartesianPath::preInsertPose(socket, {1.0, 0.0, 0.0}, 0.050,
                                                pre_insert, &pre_insert_error),
      "ROTATED_INSERT_PATH_CREATE", pre_insert_error);
  const Metrics rotated = runCase(pre_insert, socket, ++generation);
  check.require(rotated.success && validTimeLaw(rotated) &&
                    limitsSatisfied(rotated) &&
                    rotated.max_perpendicular_error_m < 1e-10 &&
                    rotated.rms_position_error_m < 1e-10,
                "ROTATED_INSERT_PATH_EXECUTION", rotated.timing.message);
  std::cout << "METRIC ROTATED_INSERT_PATH_EXECUTION"
            << " max_perpendicular_error_m="
            << rotated.max_perpendicular_error_m
            << " rms_position_error_m=" << rotated.rms_position_error_m
            << " max_orientation_error_rad="
            << rotated.max_orientation_error_rad << "\n";

  auto moving_goal = start;
  moving_goal.position_m[0] += 0.050;
  fr10_motion::KinematicState moving_initial;
  moving_initial.velocity_rad_s[0] = 0.020;
  moving_initial.acceleration_rad_s2[0] = 0.010;
  const Metrics moving =
      runCase(start, moving_goal, ++generation, &moving_initial);
  const auto &moving_first =
      moving.timing.trajectory.points.front().joint_state;
  check.require(moving.success &&
                    std::abs(moving_first.velocity_rad_s[0] - 0.020) < 1e-12 &&
                    std::abs(moving_first.acceleration_rad_s2[0] - 0.010) <
                        1e-12 &&
                    limitsSatisfied(moving),
                "PATH_TIME_MOVING_BOUNDARY_STATE", moving.timing.message);

  auto reverse_goal = start;
  reverse_goal.position_m[0] -= 0.050;
  const Metrics reverse =
      runCase(start, reverse_goal, ++generation, &moving_initial);
  check.require(
      !reverse.success &&
          reverse.timing.status == fr10_motion::TimeParameterizationStatus::
                                       REQUIRES_CONTROLLED_STOP &&
          reverse.timing.message.find("controlled stop") != std::string::npos,
      "PATH_TIME_REVERSAL_REQUIRES_CONTROLLED_STOP", reverse.timing.message);

  auto heterogeneous_goal = start;
  heterogeneous_goal.position_m[1] += 0.050;
  std::vector<fr10_motion::Pose> heterogeneous_cartesian_path;
  fr10_motion::CartesianPathConfig heterogeneous_path_config;
  heterogeneous_path_config.max_translation_step_m = 0.002;
  heterogeneous_path_config.max_rotation_step_rad = 0.01;
  std::string heterogeneous_error;
  const bool heterogeneous_path_ok = fr10_motion::CartesianPath::linear(
      start, heterogeneous_goal, heterogeneous_path_config,
      heterogeneous_cartesian_path, &heterogeneous_error);
  LinearPoseIK heterogeneous_provider;
  const auto heterogeneous_seed = heterogeneous_provider.solve(start, {}).state;
  fr10_motion::ContinuousIKConfig heterogeneous_ik_config;
  heterogeneous_ik_config.max_joint_step_rad = 0.05;
  heterogeneous_ik_config.max_position_error_m = 1e-10;
  heterogeneous_ik_config.max_orientation_error_rad = 1e-10;
  const auto heterogeneous_ik =
      heterogeneous_path_ok
          ? fr10_motion::ContinuousIK::solve(
                heterogeneous_cartesian_path, heterogeneous_seed,
                heterogeneous_provider, heterogeneous_ik_config)
          : fr10_motion::ContinuousIKResult{};
  fr10_motion::PathTimeParameterizerConfig heterogeneous_config;
  heterogeneous_config.sample_period_s = 0.008;
  heterogeneous_config.limits = fr10_motion_test::limits();
  heterogeneous_config.limits.max_joint_velocity_rad_s[0] = 0.010;
  const auto heterogeneous_timing =
      heterogeneous_ik.success
          ? fr10_motion::PathTimeParameterizer::parameterize(
                heterogeneous_cartesian_path, heterogeneous_ik.joint_path,
                heterogeneous_seed, heterogeneous_config, ++generation)
          : fr10_motion::PathTimeParameterizationResult{};
  check.require(
      heterogeneous_timing.success && heterogeneous_timing.duration_s < 4.0 &&
          heterogeneous_timing.diagnostics.max_joint_velocity_index == 1 &&
          heterogeneous_timing.max_joint_velocity_by_joint_rad_s[0] < 1e-12,
      "PATH_TIME_PER_JOINT_LIMIT_ISOLATION",
      heterogeneous_timing.message.empty() ? heterogeneous_error
                                           : heterogeneous_timing.message);

  std::vector<fr10_motion::Pose> canceled_cartesian_path{start, moving_goal};
  LinearPoseIK provider;
  std::vector<fr10_motion::KinematicState> canceled_joint_path{
      provider.solve(start, {}).state, provider.solve(moving_goal, {}).state};
  fr10_motion::PathTimeParameterizerConfig canceled_config;
  canceled_config.limits = fr10_motion_test::limits();
  const auto canceled = fr10_motion::PathTimeParameterizer::parameterize(
      canceled_cartesian_path, canceled_joint_path, canceled_joint_path.front(),
      canceled_config, ++generation, []() { return true; });
  check.require(!canceled.success &&
                    canceled.message.find("canceled") != std::string::npos,
                "PATH_TIME_CANCEL", "stale planning work was not canceled");

  auto capacity_config = canceled_config;
  capacity_config.max_samples = 3;
  capacity_config.max_scaling_iterations = 2;
  const auto capacity = fr10_motion::PathTimeParameterizer::parameterize(
      canceled_cartesian_path, canceled_joint_path, canceled_joint_path.front(),
      capacity_config, ++generation);
  check.require(!capacity.success &&
                    capacity.status == fr10_motion::TimeParameterizationStatus::FAILED &&
                    capacity.message.find("sample capacity") != std::string::npos,
                "PATH_TIME_FAILURE_DETAIL", capacity.message);
  auto moving_capacity_seed = canceled_joint_path.front();
  moving_capacity_seed.velocity_rad_s[0] = 0.020;
  const auto moving_capacity = fr10_motion::PathTimeParameterizer::parameterize(
      canceled_cartesian_path, canceled_joint_path, moving_capacity_seed,
      capacity_config, ++generation);
  check.require(!moving_capacity.success &&
                    moving_capacity.status ==
                        fr10_motion::TimeParameterizationStatus::REQUIRES_CONTROLLED_STOP &&
                    moving_capacity.message.find("controlled stop") !=
                        std::string::npos,
                "PATH_TIME_MOVING_EXHAUSTION_STOPS", moving_capacity.message);

  // A joint-space curve has smooth nonlinear FK. Timing piecewise Cartesian
  // chords invents acceleration/jerk jumps absent from the executed curve.
  std::vector<fr10_motion::KinematicState> curved_q;
  std::vector<fr10_motion::Pose> curved_p;
  auto smooth_fk=[](const auto &q,auto &pose) {
    pose.position_m={q.position_rad[0],.5*std::sin(4*q.position_rad[0]),0.};
    pose.orientation_quat={0.,0.,0.,1.};return true;
  };
  for(unsigned i=0;i<=20;++i) {
    fr10_motion::KinematicState q;q.position_rad[0]=i/20.;curved_q.push_back(q);
    fr10_motion::Pose pose;smooth_fk(q,pose);curved_p.push_back(pose);
  }
  auto fk_config=canceled_config;
  const auto chords=fr10_motion::PathTimeParameterizer::parameterize(curved_p,curved_q,
      curved_q.front(),fk_config,++generation);
  const auto fk_timed=fr10_motion::PathTimeParameterizer::parameterize(curved_p,curved_q,
      curved_q.front(),fk_config,++generation,{},smooth_fk);
  check.require(chords.success && fk_timed.success && fk_timed.duration_s<.9*chords.duration_s,
      "PATH_TIME_SMOOTH_FK_REMOVES_ARTIFICIAL_CHORD_JERK",fk_timed.message);
  bool exact_fk=true;
  for(const auto &point:fk_timed.trajectory.points) {
    fr10_motion::Pose expected;smooth_fk(point.joint_state,expected);
    for(unsigned axis=0;axis<3;++axis)
      exact_fk=exact_fk && std::abs(expected.position_m[axis]-point.desired_pose.position_m[axis])<1e-12;
  }
  check.require(exact_fk,"PATH_TIME_SMOOTH_FK_REFERENCE","timed poses differ from the executed joint curve");
  fk_config.policy=fr10_motion::PoseExecutionPolicy::INSERTION;
  const auto forbidden_fk=fr10_motion::PathTimeParameterizer::parameterize(curved_p,curved_q,
      curved_q.front(),fk_config,++generation,{},smooth_fk);
  check.require(!forbidden_fk.success,"PATH_TIME_INSERTION_PRESERVES_CARTESIAN_CONTRACT",forbidden_fk.message);
  fk_config.policy=fr10_motion::PoseExecutionPolicy::FREE_SPACE;
  const auto failed_fk=fr10_motion::PathTimeParameterizer::parameterize(curved_p,curved_q,
      curved_q.front(),fk_config,++generation,{},[](const auto &,auto &){return false;});
  check.require(!failed_fk.success && failed_fk.message.find("sampling failed")!=std::string::npos,
      "PATH_TIME_FK_FAILURE_REJECTS_TRAJECTORY",failed_fk.message);

  auto tiny_rotation_fk=[](const auto &q,auto &pose) {
    pose.position_m={q.position_rad[0],0.,0.};
    pose.orientation_quat=fr10_motion::math::quaternionMultiply(
        fr10_motion_test::axisAngle(2,1.7),
        fr10_motion_test::axisAngle(0,1e-6*q.position_rad[0]));
    return true;
  };
  std::vector<fr10_motion::Pose> tiny_p;
  for(const auto &q:curved_q) {fr10_motion::Pose p;tiny_rotation_fk(q,p);tiny_p.push_back(p);}
  const auto tiny_stationary=fr10_motion::PathTimeParameterizer::parameterize(tiny_p,curved_q,
      curved_q.front(),fk_config,++generation,{},tiny_rotation_fk);
  auto tiny_seed=curved_q.front();tiny_seed.velocity_rad_s[0]=1e-4;
  const auto tiny_moving=fr10_motion::PathTimeParameterizer::parameterize(tiny_p,curved_q,
      tiny_seed,fk_config,++generation,{},tiny_rotation_fk);
  check.require(tiny_stationary.success && tiny_moving.success &&
      tiny_moving.duration_s<2*tiny_stationary.duration_s &&
      tiny_moving.trajectory.points.front().joint_state.velocity_rad_s[0]==tiny_seed.velocity_rad_s[0],
      "PATH_TIME_SMALL_ROTATION_LIVE_SEED",tiny_moving.message);
  tiny_seed.velocity_rad_s[1]=1e-4;
  const auto tiny_transverse=fr10_motion::PathTimeParameterizer::parameterize(tiny_p,curved_q,
      tiny_seed,fk_config,++generation,{},tiny_rotation_fk);
  check.require(!tiny_transverse.success && tiny_transverse.status==
      fr10_motion::TimeParameterizationStatus::REQUIRES_CONTROLLED_STOP,
      "PATH_TIME_SMALL_TRANSVERSE_SEED_STOPS",tiny_transverse.message);

  fr10_motion::Pose unload_start, unload_goal;
  unload_goal.position_m[0] = .045;
  auto unload_config = canceled_config;
  unload_config.policy = fr10_motion::PoseExecutionPolicy::INSERTION;
  unload_config.linear_scale = .003/unload_config.limits.max_linear_velocity_m_s;
  unload_config.joint_velocity_scale = .025;
  unload_config.joint_acceleration_scale = .025;
  unload_config.joint_jerk_scale = .003125;
  unload_config.initial_linear_velocity_m_s = .0000025;
  unload_config.initial_linear_distance_m = .0001;
  const auto unload = runCase(unload_start, unload_goal, ++generation, nullptr, &unload_config);
  bool slow_start = unload.success;
  double free_speed = 0.;
  for (std::size_t i=1; i<unload.timing.trajectory.points.size(); ++i) {
    const auto &a=unload.timing.trajectory.points[i-1];
    const auto &b=unload.timing.trajectory.points[i];
    const double speed=(b.desired_pose.position_m[0]-a.desired_pose.position_m[0]) /
                       (b.time_from_start_s-a.time_from_start_s);
    if (b.desired_pose.position_m[0] <= .0001) slow_start &= speed <= .0000025+1e-9;
    if (a.desired_pose.position_m[0] >= .0003) free_speed=std::max(free_speed,speed);
  }
  check.require(slow_start && validTimeLaw(unload) && unload.timing.duration_s < 200. &&
      free_speed > .0015 && unload.max_perpendicular_error_m < 1e-9,
      "PATH_TIME_LOADED_START_THEN_SMOOTH_EXTRACTION",unload.timing.message);
  auto fast_unload_seed=fr10_motion::KinematicState{};
  fast_unload_seed.velocity_rad_s[0]=.00005;
  const auto fast_unload=runCase(unload_start,unload_goal,++generation,&fast_unload_seed,&unload_config);
  check.require(!fast_unload.success && fast_unload.timing.status==
      fr10_motion::TimeParameterizationStatus::REQUIRES_CONTROLLED_STOP,
      "PATH_TIME_LOADED_START_PRESERVES_MOVING_SEED_BY_STOP",fast_unload.timing.message);
  fast_unload_seed.velocity_rad_s[0]=.000001;
  const auto moving_unload=runCase(unload_start,unload_goal,++generation,&fast_unload_seed,&unload_config);
  check.require(moving_unload.success && moving_unload.timing.trajectory.points.front().joint_state.velocity_rad_s[0]==.000001,
      "PATH_TIME_LOADED_START_PRESERVES_ADMISSIBLE_MOVING_SEED",moving_unload.timing.message);
  unload_config.initial_linear_velocity_m_s=0.;
  check.require(!runCase(unload_start,unload_goal,++generation,nullptr,&unload_config).success,
      "PATH_TIME_LOADED_START_INVALID_PAIR_REJECTED","unpaired loaded-start configuration accepted");
  return check.result();
}
