#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_core/continuous_ik.hpp"
#include "fr10_motion_core/fr10_analytical_ik.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "test_support.hpp"

namespace fr10_motion_test {

constexpr double kPi = 3.141592653589793238462643383279502884;

inline fr10_motion::Fr10AnalyticalIKConfig analyticalConfig() {
  fr10_motion::Fr10AnalyticalIKConfig config;
  config.limits = limits();
  config.wrist_min_abs_rad = 8.0 * fr10_motion::kDegToRad;
  config.elbow_min_abs_rad = 15.0 * fr10_motion::kDegToRad;
  config.max_wrist_reach_m = 1.30;
  config.min_wrist_radius_m = 0.200;
  config.max_jacobian_condition = 30.0;
  return config;
}

inline fr10_motion::KinematicState nominalState() {
  fr10_motion::KinematicState state;
  state.position_rad =
      {2.1431, -1.9512, 1.9526, -1.6253, -1.3668, 0.0};
  return state;
}

inline std::array<double, 4> quaternionMultiply(
    const std::array<double, 4> &left,
    const std::array<double, 4> &right) {
  const auto result = std::array<double, 4>{
      left[3] * right[0] + left[0] * right[3] +
          left[1] * right[2] - left[2] * right[1],
      left[3] * right[1] - left[0] * right[2] +
          left[1] * right[3] + left[2] * right[0],
      left[3] * right[2] + left[0] * right[1] -
          left[1] * right[0] + left[2] * right[3],
      left[3] * right[3] - left[0] * right[0] -
          left[1] * right[1] - left[2] * right[2]};
  return fr10_motion::math::normalizedQuaternion(result);
}

inline std::array<double, 4> rpyQuaternion(double roll, double pitch,
                                           double yaw) {
  const auto qx = axisAngle(0, roll);
  const auto qy = axisAngle(1, pitch);
  const auto qz = axisAngle(2, yaw);
  return quaternionMultiply(qz, quaternionMultiply(qy, qx));
}

inline double positionDistance(const fr10_motion::Pose &first,
                               const fr10_motion::Pose &second) {
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = first.position_m[axis] - second.position_m[axis];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

struct ExecutedPathMetrics {
  bool success{false};
  std::string message;
  std::size_t cartesian_waypoints{0};
  std::size_t timed_samples{0};
  double max_perpendicular_error_m{0.0};
  double rms_perpendicular_error_m{0.0};
  double max_cartesian_error_m{0.0};
  double rms_cartesian_error_m{0.0};
  double endpoint_error_m{0.0};
  double max_orientation_error_rad{0.0};
  double max_joint_step_rad{0.0};
  double min_path_parameter{1.0};
  double max_path_parameter{0.0};
  fr10_motion::ContinuousIKResult ik;
  fr10_motion::PathTimeParameterizationResult timing;
};

inline ExecutedPathMetrics executeAnalyticalPath(
    const fr10_motion::Pose &start, const fr10_motion::Pose &goal,
    const fr10_motion::KinematicState &seed,
    const fr10_motion::CartesianPathConfig &path_config =
        fr10_motion::CartesianPathConfig{}) {
  ExecutedPathMetrics metrics;
  fr10_motion::Fr10AnalyticalIKProvider provider(analyticalConfig());
  std::vector<fr10_motion::Pose> cartesian_path;
  if (!fr10_motion::CartesianPath::linear(
          start, goal, path_config, cartesian_path, &metrics.message)) {
    return metrics;
  }
  metrics.cartesian_waypoints = cartesian_path.size();

  fr10_motion::ContinuousIKConfig ik_config;
  ik_config.max_joint_step_rad = 0.35;
  ik_config.max_position_error_m = 1e-6;
  ik_config.max_orientation_error_rad = 1e-6;
  metrics.ik = fr10_motion::ContinuousIK::solve(
      cartesian_path, seed, provider, ik_config);
  if (!metrics.ik.success) {
    metrics.message = metrics.ik.message;
    return metrics;
  }
  for (std::size_t index = 1; index < metrics.ik.joint_path.size(); ++index) {
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      metrics.max_joint_step_rad = std::max(
          metrics.max_joint_step_rad,
          std::abs(metrics.ik.joint_path[index].position_rad[joint] -
                   metrics.ik.joint_path[index - 1].position_rad[joint]));
    }
  }

  fr10_motion::PathTimeParameterizerConfig timing_config;
  timing_config.sample_period_s = 0.008;
  timing_config.limits = limits();
  timing_config.joint_velocity_scale = 0.5;
  timing_config.joint_acceleration_scale = 0.3;
  timing_config.joint_jerk_scale = 0.15;
  timing_config.linear_scale = 0.10;
  timing_config.angular_scale = 0.10;
  metrics.timing = fr10_motion::PathTimeParameterizer::parameterize(
      cartesian_path, metrics.ik.joint_path, seed, timing_config, 1);
  if (!metrics.timing.success) {
    metrics.message = metrics.timing.message;
    return metrics;
  }
  metrics.timed_samples = metrics.timing.trajectory.points.size();
  double perpendicular_squared_sum = 0.0;
  double cartesian_squared_sum = 0.0;
  for (const auto &point : metrics.timing.trajectory.points) {
    fr10_motion::Pose actual;
    if (!provider.forward(point.joint_state, actual, &metrics.message)) {
      return metrics;
    }
    const double cartesian_error =
        positionDistance(actual, point.desired_pose);
    const double perpendicular =
        fr10_motion::CartesianPath::translationDeviation(start, goal, actual);
    const double orientation =
        fr10_motion::math::quaternionAngularDistance(
            actual.orientation_quat, point.desired_pose.orientation_quat);
    metrics.max_cartesian_error_m =
        std::max(metrics.max_cartesian_error_m, cartesian_error);
    metrics.max_perpendicular_error_m =
        std::max(metrics.max_perpendicular_error_m, perpendicular);
    metrics.max_orientation_error_rad =
        std::max(metrics.max_orientation_error_rad, orientation);
    cartesian_squared_sum += cartesian_error * cartesian_error;
    perpendicular_squared_sum += perpendicular * perpendicular;
    const double progress =
        fr10_motion::CartesianPath::closestTranslationParameter(
            start, goal, actual);
    metrics.min_path_parameter = std::min(metrics.min_path_parameter, progress);
    metrics.max_path_parameter = std::max(metrics.max_path_parameter, progress);
  }
  if (metrics.timed_samples == 0) {
    metrics.message = "time parameterizer returned no samples";
    return metrics;
  }
  metrics.rms_cartesian_error_m = std::sqrt(
      cartesian_squared_sum / static_cast<double>(metrics.timed_samples));
  metrics.rms_perpendicular_error_m = std::sqrt(
      perpendicular_squared_sum / static_cast<double>(metrics.timed_samples));
  fr10_motion::Pose endpoint;
  if (!provider.forward(metrics.timing.trajectory.points.back().joint_state,
                        endpoint, &metrics.message)) {
    return metrics;
  }
  metrics.endpoint_error_m = positionDistance(endpoint, goal);
  metrics.success = true;
  metrics.message = "success";
  return metrics;
}

inline void printPathMetrics(const std::string &id,
                             const ExecutedPathMetrics &metrics) {
  std::cout << "METRIC " << id
            << " waypoints=" << metrics.cartesian_waypoints
            << " samples=" << metrics.timed_samples
            << " e_perp_max_m=" << metrics.max_perpendicular_error_m
            << " e_perp_rms_m=" << metrics.rms_perpendicular_error_m
            << " cartesian_max_m=" << metrics.max_cartesian_error_m
            << " cartesian_rms_m=" << metrics.rms_cartesian_error_m
            << " endpoint_error_m=" << metrics.endpoint_error_m
            << " orientation_max_rad=" << metrics.max_orientation_error_rad
            << " max_joint_step_rad=" << metrics.max_joint_step_rad
            << " message=" << metrics.message << "\n";
}

struct JacobianMetrics {
  bool valid{false};
  double sigma_min{0.0};
  double sigma_max{0.0};
  double condition{std::numeric_limits<double>::infinity()};
};

inline JacobianMetrics finiteDifferenceJacobian(
    const fr10_motion::Fr10AnalyticalIKProvider &provider,
    const fr10_motion::KinematicState &state) {
  constexpr double epsilon = 1e-7;
  Eigen::Matrix<double, 6, 6> jacobian;
  fr10_motion::Pose reference;
  if (!provider.forward(state, reference, nullptr)) {
    return {};
  }
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    auto perturbed = state;
    perturbed.position_rad[joint] += epsilon;
    fr10_motion::Pose pose;
    if (!provider.forward(perturbed, pose, nullptr)) {
      return {};
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      jacobian(static_cast<Eigen::Index>(axis),
               static_cast<Eigen::Index>(joint)) =
          (pose.position_m[axis] - reference.position_m[axis]) / epsilon;
    }
    auto relative = quaternionMultiply(
        {-reference.orientation_quat[0], -reference.orientation_quat[1],
         -reference.orientation_quat[2], reference.orientation_quat[3]},
        pose.orientation_quat);
    const auto rotation_vector =
        fr10_motion::math::quaternionToRotationVector(relative);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      jacobian(static_cast<Eigen::Index>(axis + 3),
               static_cast<Eigen::Index>(joint)) =
          rotation_vector[axis] / epsilon;
    }
  }
  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>,
                         Eigen::NoQRPreconditioner>
      svd(jacobian);
  JacobianMetrics result;
  result.sigma_max = svd.singularValues()(0);
  result.sigma_min = svd.singularValues()(5);
  result.condition = result.sigma_min > 0.0
                         ? result.sigma_max / result.sigma_min
                         : std::numeric_limits<double>::infinity();
  result.valid = jacobian.allFinite() && std::isfinite(result.condition);
  return result;
}

} // namespace fr10_motion_test
