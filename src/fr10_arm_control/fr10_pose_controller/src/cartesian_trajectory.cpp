#include "fr10_pose_controller/cartesian_trajectory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

#include "fr10_pose_controller/cartesian_path.hpp"
#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion {

const char *poseExecutionPolicyName(PoseExecutionPolicy policy) {
  switch (policy) {
  case PoseExecutionPolicy::FREE_SPACE:
    return "FREE_SPACE";
  case PoseExecutionPolicy::INSERTION:
    return "INSERTION";
  }
  return "UNKNOWN";
}

const char *timeParameterConstraintName(TimeParameterConstraint constraint) {
  switch (constraint) {
  case TimeParameterConstraint::NONE:
    return "NONE";
  case TimeParameterConstraint::JOINT_VELOCITY_SLOPE:
    return "JOINT_SLOPE (dq/ds)";
  case TimeParameterConstraint::JOINT_ACCELERATION:
    return "JOINT_ACCELERATION";
  case TimeParameterConstraint::JOINT_JERK:
    return "JOINT_JERK";
  case TimeParameterConstraint::LINEAR_VELOCITY_DERIVATIVE:
    return "LINEAR_PATH_DERIVATIVE (dx/ds)";
  case TimeParameterConstraint::LINEAR_ACCELERATION:
    return "LINEAR_ACCELERATION";
  case TimeParameterConstraint::LINEAR_JERK:
    return "LINEAR_JERK";
  case TimeParameterConstraint::ANGULAR_VELOCITY_DERIVATIVE:
    return "ANGULAR_SLERP_DERIVATIVE (dtheta/ds)";
  case TimeParameterConstraint::ANGULAR_ACCELERATION:
    return "ANGULAR_ACCELERATION";
  case TimeParameterConstraint::ANGULAR_JERK:
    return "ANGULAR_JERK";
  }
  return "UNKNOWN";
}

namespace {

#include "cartesian_trajectory_detail.inc"

struct DerivativeMaxima {
  double velocity{0.0};
  double acceleration{0.0};
  double jerk{0.0};
};

struct TranslationDifference {
  using Derivative = std::array<double, 3>;

  Derivative first(const Derivative &current, const Derivative &previous,
                   double dt) const {
    Derivative result{};
    for (std::size_t axis = 0; axis < result.size(); ++axis) {
      result[axis] = (current[axis] - previous[axis]) / dt;
    }
    return result;
  }

  double magnitude(const Derivative &value) const { return norm3(value); }
};

struct QuaternionDifference {
  using Derivative = double;

  double first(const std::array<double, 4> &current,
               const std::array<double, 4> &previous, double dt) const {
    return math::quaternionAngularDistance(previous, current) / dt;
  }

  double first(double current, double previous, double dt) const {
    return (current - previous) / dt;
  }

  double magnitude(double value) const { return std::abs(value); }
};

template <typename T, typename DiffOp>
bool checkDerivativeLimits(
    const std::vector<double> &times, const std::vector<T> &values,
    DiffOp diff_op, double max_velocity, double max_acceleration,
    double max_jerk, const typename DiffOp::Derivative &initial_velocity,
    const typename DiffOp::Derivative &initial_acceleration,
    DerivativeMaxima &maximum) {
  if (times.size() != values.size() || times.size() < 2) {
    return false;
  }
  auto previous_velocity = initial_velocity;
  auto previous_acceleration = initial_acceleration;
  for (std::size_t index = 1; index < times.size(); ++index) {
    const double dt = times[index] - times[index - 1];
    if (!(dt > 0.0) || !std::isfinite(dt)) {
      return false;
    }
    const auto velocity = diff_op.first(values[index], values[index - 1], dt);
    const auto acceleration =
        diff_op.first(velocity, previous_velocity, dt);
    const auto jerk =
        diff_op.first(acceleration, previous_acceleration, dt);
    maximum.velocity =
        std::max(maximum.velocity, diff_op.magnitude(velocity));
    maximum.acceleration =
        std::max(maximum.acceleration, diff_op.magnitude(acceleration));
    maximum.jerk = std::max(maximum.jerk, diff_op.magnitude(jerk));
    previous_velocity = velocity;
    previous_acceleration = acceleration;
  }
  constexpr double kNumericalSlack = 1e-7;
  return maximum.velocity <= max_velocity + kNumericalSlack &&
         maximum.acceleration <= max_acceleration + kNumericalSlack &&
         maximum.jerk <= max_jerk + kNumericalSlack;
}

} // namespace

PathTimeParameterizationResult PathTimeParameterizer::parameterize(
    const std::vector<Pose> &cartesian_path,
    const std::vector<KinematicState> &joint_path,
    const KinematicState &initial_state,
    const PathTimeParameterizerConfig &config, uint64_t generation,
    const CancelCheck &cancel) {
  PathTimeParameterizationResult result;
  result.diagnostics.policy = config.policy;
  if (!inputValid(cartesian_path, joint_path, initial_state, config,
                  result.message)) {
    return result;
  }

  const CartesianPathSlopes path_slopes =
      maxCartesianPathSlopes(cartesian_path);
  const CartesianPathSlopes path_distance =
      cartesianPathDistance(cartesian_path);
  result.diagnostics.trajectory_distance_m = path_distance.translation;
  result.diagnostics.trajectory_rotation_rad = path_distance.rotation;
  result.diagnostics.max_translation_derivative_m = path_slopes.translation;
  result.diagnostics.max_rotation_derivative_rad = path_slopes.rotation;
  if (!std::isfinite(path_slopes.translation) ||
      !std::isfinite(path_slopes.rotation)) {
    result.message = "Cartesian path has an invalid angular distance";
    return result;
  }

  const auto seed_spline = buildSpline(joint_path, initial_state, 0.0);
  const ScalarLimits seed_scalar_limits =
      initialScalarLimits(seed_spline, cartesian_path, config);
  const MovingBoundaryProjection moving_boundary =
      projectMovingBoundary(joint_path, initial_state);
  if (moving_boundary.moving &&
      (!moving_boundary.compatible ||
       moving_boundary.path_velocity >
           seed_scalar_limits.velocity * (1.0 + 1e-6))) {
    result.status = TimeParameterizationStatus::REQUIRES_CONTROLLED_STOP;
    result.diagnostics.initial_velocity_constraint =
        seed_scalar_limits.velocity_constraint;
    result.message =
        "Moving state is incompatible with the new path direction/limits; "
        "controlled stop and zero-velocity replan required";
    return result;
  }
  const double initial_path_velocity =
      moving_boundary.moving ? moving_boundary.path_velocity : 0.0;
  const auto spline = buildSpline(joint_path, initial_state, initial_path_velocity);
  const ScalarLimits unscaled_limits = initialScalarLimits(spline, cartesian_path, config);

  for (std::size_t iteration = 0; iteration < config.max_scaling_iterations;
       ++iteration) {
    if (cancel && cancel()) {
      result.status = TimeParameterizationStatus::CANCELED;
      result.message = "Path time parameterization canceled";
      return result;
    }
    const double timing_scale = std::pow(0.8, static_cast<double>(iteration));
    ScalarLimits scalar_limits = unscaled_limits;
    scalar_limits.velocity *= timing_scale;
    scalar_limits.acceleration *= timing_scale * timing_scale;
    scalar_limits.jerk *= timing_scale * timing_scale * timing_scale;
    if (moving_boundary.moving) {
      scalar_limits.velocity =
          std::max(scalar_limits.velocity, 1.01 * initial_path_velocity);
    }

    ruckig::Ruckig<1> otg(config.sample_period_s);
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;
    input.current_position[0] = 0.0;
    input.current_velocity[0] = initial_path_velocity;
    input.current_acceleration[0] = 0.0;
    input.target_position[0] = 1.0;
    input.target_velocity[0] = 0.0;
    input.target_acceleration[0] = 0.0;
    input.max_velocity[0] = scalar_limits.velocity;
    input.max_acceleration[0] = scalar_limits.acceleration;
    input.max_jerk[0] = scalar_limits.jerk;
    input.minimum_duration = 2.0 * config.minimum_ramp_time_s;
    input.duration_discretization = ruckig::DurationDiscretization::Discrete;

    PathTimeParameterizationResult candidate;
    candidate.diagnostics.policy = config.policy;
    candidate.diagnostics.trajectory_distance_m = path_distance.translation;
    candidate.diagnostics.trajectory_rotation_rad = path_distance.rotation;
    candidate.diagnostics.max_translation_derivative_m =
        path_slopes.translation;
    candidate.diagnostics.max_rotation_derivative_rad = path_slopes.rotation;
    candidate.diagnostics.max_joint_path_slope_rad = maxSplineSlopes(spline);
    candidate.diagnostics.initial_velocity_constraint =
        scalar_limits.velocity_constraint;
    candidate.diagnostics.scaling_iterations = iteration + 1;
    candidate.trajectory.generation = generation;
    candidate.trajectory.seed_joint_state = initial_state;
    candidate.trajectory.path_start = cartesian_path.front();
    candidate.trajectory.path_goal = cartesian_path.back();
    candidate.trajectory.points.reserve(
        std::min<std::size_t>(config.max_samples, 4096));

    CartesianTrajectoryPoint initial_point;
    initial_point.joint_state = initial_state;
    initial_point.desired_pose = cartesian_path.front();
    initial_point.desired_translation_tangent =
        CartesianPath::translationTangent(cartesian_path, 0.0);
    initial_point.path_parameter = 0.0;
    initial_point.time_from_start_s = 0.0;
    candidate.trajectory.points.push_back(initial_point);

    std::array<double, kJointCount> previous_acceleration =
        initial_state.acceleration_rad_s2;
    double elapsed_s = 0.0;
    bool generation_ok = true;
    while (candidate.trajectory.points.size() < config.max_samples) {
      if (cancel && cancel()) {
        candidate.status = TimeParameterizationStatus::CANCELED;
        candidate.message = "Path time parameterization canceled";
        return candidate;
      }
      const ruckig::Result update_result = otg.update(input, output);
      if (update_result < 0) {
        generation_ok = false;
        candidate.message = "Ruckig failed to generate scalar path timing";
        break;
      }
      elapsed_s += config.sample_period_s;
      const double s = std::clamp(output.new_position[0], 0.0, 1.0);
      const double s_velocity = output.new_velocity[0];
      const double s_acceleration = output.new_acceleration[0];
      const auto evaluated = evaluateSpline(spline, s);

      CartesianTrajectoryPoint point;
      point.path_parameter = s;
      point.time_from_start_s = elapsed_s;
      point.desired_pose = CartesianPath::interpolate(cartesian_path, s);
      point.desired_translation_tangent =
          CartesianPath::translationTangent(cartesian_path, s);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        point.joint_state.position_rad[joint] = evaluated.position[joint];
        point.joint_state.velocity_rad_s[joint] =
            evaluated.first_derivative[joint] * s_velocity;
        point.joint_state.acceleration_rad_s2[joint] =
            evaluated.second_derivative[joint] * s_velocity * s_velocity +
            evaluated.first_derivative[joint] * s_acceleration;
        if (!std::isfinite(point.joint_state.position_rad[joint]) ||
            !std::isfinite(point.joint_state.velocity_rad_s[joint]) ||
            !std::isfinite(point.joint_state.acceleration_rad_s2[joint]) ||
            point.joint_state.position_rad[joint] <
                config.limits.min_joint_position_rad[joint] ||
            point.joint_state.position_rad[joint] >
                config.limits.max_joint_position_rad[joint]) {
          generation_ok = false;
          candidate.message =
              "Time-parameterized spline violates a joint position limit";
          break;
        }
        candidate.max_joint_velocity_rad_s =
            std::max(candidate.max_joint_velocity_rad_s,
                     std::abs(point.joint_state.velocity_rad_s[joint]));
        candidate.max_joint_velocity_by_joint_rad_s[joint] =
            std::max(candidate.max_joint_velocity_by_joint_rad_s[joint],
                     std::abs(point.joint_state.velocity_rad_s[joint]));
        candidate.max_joint_acceleration_rad_s2 =
            std::max(candidate.max_joint_acceleration_rad_s2,
                     std::abs(point.joint_state.acceleration_rad_s2[joint]));
        candidate.max_joint_acceleration_by_joint_rad_s2[joint] =
            std::max(candidate.max_joint_acceleration_by_joint_rad_s2[joint],
                     std::abs(point.joint_state.acceleration_rad_s2[joint]));
        const double joint_jerk =
            std::abs(point.joint_state.acceleration_rad_s2[joint] -
                     previous_acceleration[joint]) /
            config.sample_period_s;
        candidate.max_joint_jerk_rad_s3 =
            std::max(candidate.max_joint_jerk_rad_s3, joint_jerk);
        candidate.max_joint_jerk_by_joint_rad_s3[joint] = std::max(
            candidate.max_joint_jerk_by_joint_rad_s3[joint], joint_jerk);
      }
      if (!generation_ok) {
        break;
      }

      candidate.trajectory.points.push_back(point);
      previous_acceleration = point.joint_state.acceleration_rad_s2;
      output.pass_to_input(input);
      if (update_result == ruckig::Result::Finished) {
        break;
      }
    }

    if (candidate.trajectory.points.size() >= config.max_samples) {
      generation_ok = false;
      candidate.message =
          "Time-parameterized trajectory exceeds sample capacity";
    }
    if (!generation_ok) {
      if (candidate.message.find("position limit") != std::string::npos) {
        return candidate;
      }
      continue;
    }

    std::vector<double> sample_times;
    std::vector<std::array<double, 3>> translations;
    std::vector<std::array<double, 4>> orientations;
    sample_times.reserve(candidate.trajectory.points.size());
    translations.reserve(candidate.trajectory.points.size());
    orientations.reserve(candidate.trajectory.points.size());
    for (const auto &point : candidate.trajectory.points) {
      sample_times.push_back(point.time_from_start_s);
      translations.push_back(point.desired_pose.position_m);
      orientations.push_back(point.desired_pose.orientation_quat);
    }
    std::array<double, 3> initial_linear_velocity{};
    double initial_angular_velocity = 0.0;
    if (moving_boundary.moving) {
      const double parameter_scale =
          static_cast<double>(cartesian_path.size() - 1);
      for (std::size_t axis = 0; axis < 3; ++axis) {
        initial_linear_velocity[axis] =
            (cartesian_path[1].position_m[axis] -
             cartesian_path[0].position_m[axis]) *
            parameter_scale * initial_path_velocity;
      }
      initial_angular_velocity = math::quaternionAngularDistance(
                                     cartesian_path[0].orientation_quat,
                                     cartesian_path[1].orientation_quat) *
                                 parameter_scale * initial_path_velocity;
    }
    DerivativeMaxima linear_metrics;
    DerivativeMaxima angular_metrics;
    const bool linear_derivatives_valid = checkDerivativeLimits(
        sample_times, translations, TranslationDifference{},
        config.limits.max_linear_velocity_m_s * config.linear_scale,
        config.limits.max_linear_acceleration_m_s2 *
            std::sqrt(config.linear_scale),
        config.limits.max_linear_jerk_m_s3 * config.linear_scale,
        initial_linear_velocity, std::array<double, 3>{}, linear_metrics);
    const bool angular_derivatives_valid = checkDerivativeLimits(
        sample_times, orientations, QuaternionDifference{},
        config.limits.max_angular_velocity_rad_s * config.angular_scale,
        config.limits.max_angular_acceleration_rad_s2 *
            std::sqrt(config.angular_scale),
        config.limits.max_angular_jerk_rad_s3 * config.angular_scale,
        initial_angular_velocity, 0.0, angular_metrics);
    candidate.max_linear_velocity_m_s = linear_metrics.velocity;
    candidate.max_linear_acceleration_m_s2 = linear_metrics.acceleration;
    candidate.max_linear_jerk_m_s3 = linear_metrics.jerk;
    candidate.max_angular_velocity_rad_s = angular_metrics.velocity;
    candidate.max_angular_acceleration_rad_s2 = angular_metrics.acceleration;
    candidate.max_angular_jerk_rad_s3 = angular_metrics.jerk;
    candidate.duration_s = elapsed_s;
    if (linear_derivatives_valid && angular_derivatives_valid &&
        metricsWithinLimits(candidate, config)) {
      candidate.success = true;
      candidate.status = TimeParameterizationStatus::SUCCESS;
      candidate.message = "success";
      return candidate;
    }
    result = std::move(candidate);
  }

  result.success = false;
  result.status = TimeParameterizationStatus::FAILED;
  result.message =
      "Unable to satisfy path timing limits within scaling iterations";
  return result;
}

} // namespace fr10_motion
