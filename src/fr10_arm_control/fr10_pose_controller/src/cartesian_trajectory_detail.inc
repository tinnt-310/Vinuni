constexpr double kMinimumLimit = 1e-6;
constexpr double kStateTolerance = 1e-9;

template <std::size_t N> bool finiteArray(const std::array<double, N> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool validScale(double value) {
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

bool finitePose(const Pose &pose) {
  return finiteArray(pose.position_m) && finiteArray(pose.orientation_quat) &&
         math::quaternionNormSquared(pose.orientation_quat) > 1e-12;
}

struct QuinticSegment {
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> coefficient{
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero()};
  double start_s{0.0};
  double width_s{1.0};
};

struct SplineSample {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> first_derivative{};
  std::array<double, kJointCount> second_derivative{};
};

std::vector<QuinticSegment>
buildSpline(const std::vector<KinematicState> &joint_path,
            const KinematicState &initial_state, double initial_path_velocity) {
  const std::size_t waypoint_count = joint_path.size();
  const double step_s = 1.0 / static_cast<double>(waypoint_count - 1);
  std::vector<std::array<double, kJointCount>> first(waypoint_count);
  std::vector<std::array<double, kJointCount>> second(waypoint_count);

  for (std::size_t waypoint = 0; waypoint < waypoint_count; ++waypoint) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (waypoint == 0) {
        first[waypoint][joint] = (joint_path[1].position_rad[joint] -
                                  joint_path[0].position_rad[joint]) /
                                 step_s;
        second[waypoint][joint] =
            waypoint_count > 2 ? (joint_path[2].position_rad[joint] -
                                  2.0 * joint_path[1].position_rad[joint] +
                                  joint_path[0].position_rad[joint]) /
                                     (step_s * step_s)
                               : 0.0;
      } else if (waypoint + 1 == waypoint_count) {
        first[waypoint][joint] =
            (joint_path[waypoint].position_rad[joint] -
             joint_path[waypoint - 1].position_rad[joint]) /
            step_s;
        second[waypoint][joint] =
            waypoint_count > 2
                ? (joint_path[waypoint].position_rad[joint] -
                   2.0 * joint_path[waypoint - 1].position_rad[joint] +
                   joint_path[waypoint - 2].position_rad[joint]) /
                      (step_s * step_s)
                : 0.0;
      } else {
        first[waypoint][joint] =
            (joint_path[waypoint + 1].position_rad[joint] -
             joint_path[waypoint - 1].position_rad[joint]) /
            (2.0 * step_s);
        second[waypoint][joint] =
            (joint_path[waypoint + 1].position_rad[joint] -
             2.0 * joint_path[waypoint].position_rad[joint] +
             joint_path[waypoint - 1].position_rad[joint]) /
            (step_s * step_s);
      }
    }
  }

  if (initial_path_velocity > kStateTolerance) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      first.front()[joint] =
          initial_state.velocity_rad_s[joint] / initial_path_velocity;
      second.front()[joint] = initial_state.acceleration_rad_s2[joint] /
                              (initial_path_velocity * initial_path_velocity);
    }
  }

  std::vector<QuinticSegment> segments;
  segments.reserve(waypoint_count - 1);
  for (std::size_t segment_index = 0; segment_index + 1 < waypoint_count;
       ++segment_index) {
    QuinticSegment segment;
    segment.start_s = static_cast<double>(segment_index) * step_s;
    segment.width_s = step_s;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double p0 = joint_path[segment_index].position_rad[joint];
      const double p1 = joint_path[segment_index + 1].position_rad[joint];
      const double v0 = first[segment_index][joint] * step_s;
      const double v1 = first[segment_index + 1][joint] * step_s;
      const double a0 = second[segment_index][joint] * step_s * step_s;
      const double a1 = second[segment_index + 1][joint] * step_s * step_s;
      const double displacement = p1 - p0 - v0 - 0.5 * a0;
      const double velocity_residual = v1 - v0 - a0;
      const double acceleration_residual = a1 - a0;
      segment.coefficient(joint, 0) = p0;
      segment.coefficient(joint, 1) = v0;
      segment.coefficient(joint, 2) = 0.5 * a0;
      segment.coefficient(joint, 3) = 10.0 * displacement -
                                      4.0 * velocity_residual +
                                      0.5 * acceleration_residual;
      segment.coefficient(joint, 4) = -15.0 * displacement +
                                      7.0 * velocity_residual -
                                      acceleration_residual;
      segment.coefficient(joint, 5) = 6.0 * displacement -
                                      3.0 * velocity_residual +
                                      0.5 * acceleration_residual;
    }
    segments.push_back(segment);
  }
  return segments;
}

SplineSample evaluateSpline(const std::vector<QuinticSegment> &segments,
                            double path_parameter) {
  path_parameter = std::clamp(path_parameter, 0.0, 1.0);
  const double scaled = path_parameter * static_cast<double>(segments.size());
  const std::size_t segment_index = std::min<std::size_t>(
      static_cast<std::size_t>(scaled), segments.size() - 1);
  const auto &segment = segments[segment_index];
  const double u = std::clamp(
      (path_parameter - segment.start_s) / segment.width_s, 0.0, 1.0);
  SplineSample result;
  const Eigen::Matrix<double, 6, 1> position =
      segment.coefficient.col(0) +
      u * (segment.coefficient.col(1) +
           u * (segment.coefficient.col(2) +
                u * (segment.coefficient.col(3) +
                     u * (segment.coefficient.col(4) +
                          u * segment.coefficient.col(5)))));
  const Eigen::Matrix<double, 6, 1> derivative_u =
      segment.coefficient.col(1) +
      u * (2.0 * segment.coefficient.col(2) +
           u * (3.0 * segment.coefficient.col(3) +
                u * (4.0 * segment.coefficient.col(4) +
                     u * 5.0 * segment.coefficient.col(5))));
  const Eigen::Matrix<double, 6, 1> second_derivative_u =
      2.0 * segment.coefficient.col(2) +
      u * (6.0 * segment.coefficient.col(3) +
           u * (12.0 * segment.coefficient.col(4) +
                u * 20.0 * segment.coefficient.col(5)));
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(result.position.data()) = position;
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(result.first_derivative.data()) =
      derivative_u / segment.width_s;
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(result.second_derivative.data()) =
      second_derivative_u / (segment.width_s * segment.width_s);
  return result;
}

std::array<double, kJointCount>
maxSplineSlopes(const std::vector<QuinticSegment> &segments) {
  std::array<double, kJointCount> maximum{};
  constexpr std::size_t kSamplesPerSegment = 12;
  for (std::size_t segment = 0; segment < segments.size(); ++segment) {
    for (std::size_t sample = 0; sample <= kSamplesPerSegment; ++sample) {
      const double s = (static_cast<double>(segment) +
                        static_cast<double>(sample) /
                            static_cast<double>(kSamplesPerSegment)) /
                       static_cast<double>(segments.size());
      const auto evaluated = evaluateSpline(segments, s);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        maximum[joint] = std::max(maximum[joint],
                                  std::abs(evaluated.first_derivative[joint]));
      }
    }
  }
  return maximum;
}

struct ScalarLimits {
  double velocity{std::numeric_limits<double>::infinity()};
  double acceleration{std::numeric_limits<double>::infinity()};
  double jerk{std::numeric_limits<double>::infinity()};
  TimeParameterConstraint velocity_constraint{TimeParameterConstraint::NONE};
};

struct CartesianPathSlopes {
  double translation{0.0};
  double rotation{0.0};
};

struct MovingBoundaryProjection {
  bool moving{false};
  bool compatible{true};
  double path_velocity{0.0};
  double residual_rad_s{0.0};
  double joint_velocity_norm_rad_s{0.0};
};

MovingBoundaryProjection
projectMovingBoundary(const std::vector<KinematicState> &joint_path,
                      const KinematicState &initial_state) {
  MovingBoundaryProjection projection;
  const double parameter_scale = static_cast<double>(joint_path.size() - 1);
  double tangent_squared = 0.0;
  double velocity_dot_tangent = 0.0;
  double acceleration_squared = 0.0;
  std::array<double, kJointCount> tangent{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    tangent[joint] = (joint_path[1].position_rad[joint] -
                      joint_path[0].position_rad[joint]) *
                     parameter_scale;
    tangent_squared += tangent[joint] * tangent[joint];
    velocity_dot_tangent +=
        initial_state.velocity_rad_s[joint] * tangent[joint];
    projection.joint_velocity_norm_rad_s +=
        initial_state.velocity_rad_s[joint] *
        initial_state.velocity_rad_s[joint];
    acceleration_squared += initial_state.acceleration_rad_s2[joint] *
                            initial_state.acceleration_rad_s2[joint];
  }
  projection.joint_velocity_norm_rad_s =
      std::sqrt(projection.joint_velocity_norm_rad_s);
  const double acceleration_norm = std::sqrt(acceleration_squared);
  projection.moving =
      projection.joint_velocity_norm_rad_s > 1e-5 || acceleration_norm > 1e-4;
  if (!projection.moving) {
    return projection;
  }
  if (projection.joint_velocity_norm_rad_s <= 1e-5 ||
      tangent_squared <= 1e-12) {
    projection.compatible = false;
    return projection;
  }
  projection.path_velocity = velocity_dot_tangent / tangent_squared;
  double residual_squared = 0.0;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    const double residual = initial_state.velocity_rad_s[joint] -
                            tangent[joint] * projection.path_velocity;
    residual_squared += residual * residual;
  }
  projection.residual_rad_s = std::sqrt(residual_squared);
  const double residual_limit =
      std::max(1e-3, 0.10 * projection.joint_velocity_norm_rad_s);
  projection.compatible = projection.path_velocity > 1e-5 &&
                          projection.residual_rad_s <= residual_limit;
  return projection;
}

CartesianPathSlopes
maxCartesianPathSlopes(const std::vector<Pose> &cartesian_path) {
  CartesianPathSlopes result;
  if (cartesian_path.size() < 2) {
    return result;
  }
  const double parameter_scale = static_cast<double>(cartesian_path.size() - 1);
  for (std::size_t index = 1; index < cartesian_path.size(); ++index) {
    double translation_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = cartesian_path[index].position_m[axis] -
                           cartesian_path[index - 1].position_m[axis];
      translation_squared += delta * delta;
    }
    result.translation = std::max(
        result.translation, std::sqrt(translation_squared) * parameter_scale);
    result.rotation = std::max(result.rotation,
                               math::quaternionAngularDistance(
                                   cartesian_path[index - 1].orientation_quat,
                                   cartesian_path[index].orientation_quat) *
                                   parameter_scale);
  }
  return result;
}

CartesianPathSlopes
cartesianPathDistance(const std::vector<Pose> &cartesian_path) {
  CartesianPathSlopes result;
  for (std::size_t index = 1; index < cartesian_path.size(); ++index) {
    double translation_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = cartesian_path[index].position_m[axis] -
                           cartesian_path[index - 1].position_m[axis];
      translation_squared += delta * delta;
    }
    result.translation += std::sqrt(translation_squared);
    result.rotation += math::quaternionAngularDistance(
        cartesian_path[index - 1].orientation_quat,
        cartesian_path[index].orientation_quat);
  }
  return result;
}

void applyVelocityLimit(double candidate, TimeParameterConstraint constraint,
                        ScalarLimits &scalar) {
  if (candidate < scalar.velocity) {
    scalar.velocity = candidate;
    scalar.velocity_constraint = constraint;
  }
}

double norm3(const std::array<double, 3> &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

ScalarLimits initialScalarLimits(const std::vector<QuinticSegment> &segments,
                                 const std::vector<Pose> &cartesian_path,
                                 const PathTimeParameterizerConfig &config) {
  ScalarLimits scalar;
  const double linear_velocity =
      config.limits.max_linear_velocity_m_s * config.linear_scale;
  const double linear_acceleration =
      config.limits.max_linear_acceleration_m_s2 *
      std::sqrt(config.linear_scale);
  const double linear_jerk =
      config.limits.max_linear_jerk_m_s3 * config.linear_scale;
  const double angular_velocity =
      config.limits.max_angular_velocity_rad_s * config.angular_scale;
  const double angular_acceleration =
      config.limits.max_angular_acceleration_rad_s2 *
      std::sqrt(config.angular_scale);
  const double angular_jerk =
      config.limits.max_angular_jerk_rad_s3 * config.angular_scale;

  const CartesianPathSlopes path_slopes =
      maxCartesianPathSlopes(cartesian_path);
  if (path_slopes.translation > kStateTolerance) {
    applyVelocityLimit(linear_velocity / path_slopes.translation,
                       TimeParameterConstraint::LINEAR_VELOCITY_DERIVATIVE,
                       scalar);
    scalar.acceleration = std::min(
        scalar.acceleration, linear_acceleration / path_slopes.translation);
    scalar.jerk = std::min(scalar.jerk, linear_jerk / path_slopes.translation);
  }
  if (path_slopes.rotation > kStateTolerance) {
    applyVelocityLimit(angular_velocity / path_slopes.rotation,
                       TimeParameterConstraint::ANGULAR_VELOCITY_DERIVATIVE,
                       scalar);
    scalar.acceleration = std::min(scalar.acceleration,
                                   angular_acceleration / path_slopes.rotation);
    scalar.jerk = std::min(scalar.jerk, angular_jerk / path_slopes.rotation);
  }

  const auto maximum_slope = maxSplineSlopes(segments);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (maximum_slope[joint] > kStateTolerance) {
      applyVelocityLimit(config.limits.max_joint_velocity_rad_s[joint] *
                             config.joint_velocity_scale / maximum_slope[joint],
                         TimeParameterConstraint::JOINT_VELOCITY_SLOPE, scalar);
      scalar.acceleration =
          std::min(scalar.acceleration,
                   config.limits.max_joint_acceleration_rad_s2[joint] *
                       config.joint_acceleration_scale / maximum_slope[joint]);
      scalar.jerk = std::min(
          scalar.jerk, config.limits.max_joint_jerk_rad_s3[joint] *
                           config.joint_jerk_scale / maximum_slope[joint]);
    }
  }
  if (!std::isfinite(scalar.velocity)) {
    scalar.velocity = 1.0;
  }
  if (!std::isfinite(scalar.acceleration)) {
    scalar.acceleration = 1.0;
  }
  if (!std::isfinite(scalar.jerk)) {
    scalar.jerk = 1.0;
  }
  scalar.velocity = std::max(scalar.velocity, kMinimumLimit);
  scalar.acceleration = std::max(scalar.acceleration, kMinimumLimit);
  scalar.jerk = std::max(scalar.jerk, kMinimumLimit);
  return scalar;
}

bool inputValid(const std::vector<Pose> &cartesian_path,
                const std::vector<KinematicState> &joint_path,
                const KinematicState &initial_state,
                const PathTimeParameterizerConfig &config,
                std::string &message) {
  if (cartesian_path.size() < 2 || cartesian_path.size() != joint_path.size()) {
    message =
        "Cartesian and joint paths must contain the same two-or-more waypoints";
    return false;
  }
  if (!(config.sample_period_s > 0.0) ||
      !std::isfinite(config.sample_period_s) || config.max_samples < 2 ||
      config.max_scaling_iterations == 0 ||
      !std::isfinite(config.minimum_ramp_time_s) ||
      config.minimum_ramp_time_s < 0.20 ||
      !validScale(config.joint_velocity_scale) ||
      !validScale(config.joint_acceleration_scale) ||
      !validScale(config.joint_jerk_scale) ||
      !validScale(config.linear_scale) || !validScale(config.angular_scale) ||
      !(config.limits.max_linear_velocity_m_s > 0.0) ||
      !std::isfinite(config.limits.max_linear_velocity_m_s) ||
      !(config.limits.max_linear_acceleration_m_s2 > 0.0) ||
      !std::isfinite(config.limits.max_linear_acceleration_m_s2) ||
      !(config.limits.max_linear_jerk_m_s3 > 0.0) ||
      !std::isfinite(config.limits.max_linear_jerk_m_s3) ||
      !(config.limits.max_angular_velocity_rad_s > 0.0) ||
      !std::isfinite(config.limits.max_angular_velocity_rad_s) ||
      !(config.limits.max_angular_acceleration_rad_s2 > 0.0) ||
      !std::isfinite(config.limits.max_angular_acceleration_rad_s2) ||
      !(config.limits.max_angular_jerk_rad_s3 > 0.0) ||
      !std::isfinite(config.limits.max_angular_jerk_rad_s3) ||
      !finiteArray(initial_state.position_rad) ||
      !finiteArray(initial_state.velocity_rad_s) ||
      !finiteArray(initial_state.acceleration_rad_s2)) {
    message = "Path time-parameterization input/configuration is invalid";
    return false;
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (!(config.limits.max_joint_velocity_rad_s[joint] > 0.0) ||
        !std::isfinite(config.limits.max_joint_velocity_rad_s[joint]) ||
        !(config.limits.max_joint_acceleration_rad_s2[joint] > 0.0) ||
        !std::isfinite(config.limits.max_joint_acceleration_rad_s2[joint]) ||
        !(config.limits.max_joint_jerk_rad_s3[joint] > 0.0) ||
        !std::isfinite(config.limits.max_joint_jerk_rad_s3[joint]) ||
        initial_state.position_rad[joint] <
            config.limits.min_joint_position_rad[joint] ||
        initial_state.position_rad[joint] >
            config.limits.max_joint_position_rad[joint] ||
        std::abs(initial_state.velocity_rad_s[joint]) >
            config.limits.max_joint_velocity_rad_s[joint] *
                    config.joint_velocity_scale +
                1e-9 ||
        std::abs(initial_state.acceleration_rad_s2[joint]) >
            config.limits.max_joint_acceleration_rad_s2[joint] *
                    config.joint_acceleration_scale +
                1e-9) {
      message = "Initial joint state or configured joint limits are invalid";
      return false;
    }
  }
  for (std::size_t waypoint = 0; waypoint < joint_path.size(); ++waypoint) {
    if (!finitePose(cartesian_path[waypoint]) ||
        !finiteArray(joint_path[waypoint].position_rad)) {
      message = "Path contains a non-finite pose or joint waypoint";
      return false;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (joint_path[waypoint].position_rad[joint] <
              config.limits.min_joint_position_rad[joint] ||
          joint_path[waypoint].position_rad[joint] >
              config.limits.max_joint_position_rad[joint]) {
        message = "Joint path violates a configured position limit";
        return false;
      }
    }
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (std::abs(joint_path.front().position_rad[joint] -
                 initial_state.position_rad[joint]) > 1e-7) {
      message = "Joint path does not begin at the synchronized initial state";
      return false;
    }
  }
  return true;
}

double utilization(double value, double limit) {
  return limit > 0.0 ? value / limit : std::numeric_limits<double>::infinity();
}

void populateDiagnostics(PathTimeParameterizationResult &result,
                         const PathTimeParameterizerConfig &config) {
  auto &diagnostics = result.diagnostics;
  diagnostics.policy = config.policy;
  diagnostics.duration_s = result.duration_s;
  double limiting_ratio = -1.0;
  auto consider = [&](double ratio, TimeParameterConstraint constraint) {
    if (ratio > limiting_ratio) {
      limiting_ratio = ratio;
      diagnostics.limiting_constraint = constraint;
    }
  };

  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    diagnostics.joint_velocity_ratio[joint] =
        utilization(result.max_joint_velocity_by_joint_rad_s[joint],
                    config.limits.max_joint_velocity_rad_s[joint] *
                        config.joint_velocity_scale);
    diagnostics.joint_acceleration_ratio[joint] =
        utilization(result.max_joint_acceleration_by_joint_rad_s2[joint],
                    config.limits.max_joint_acceleration_rad_s2[joint] *
                        config.joint_acceleration_scale);
    diagnostics.joint_jerk_ratio[joint] = utilization(
        result.max_joint_jerk_by_joint_rad_s3[joint],
        config.limits.max_joint_jerk_rad_s3[joint] * config.joint_jerk_scale);
    if (diagnostics.joint_velocity_ratio[joint] >
        diagnostics.max_joint_velocity_ratio) {
      diagnostics.max_joint_velocity_ratio =
          diagnostics.joint_velocity_ratio[joint];
      diagnostics.max_joint_velocity_index = joint;
    }
    if (diagnostics.joint_acceleration_ratio[joint] >
        diagnostics.max_joint_acceleration_ratio) {
      diagnostics.max_joint_acceleration_ratio =
          diagnostics.joint_acceleration_ratio[joint];
      diagnostics.max_joint_acceleration_index = joint;
    }
    if (diagnostics.joint_jerk_ratio[joint] >
        diagnostics.max_joint_jerk_ratio) {
      diagnostics.max_joint_jerk_ratio = diagnostics.joint_jerk_ratio[joint];
      diagnostics.max_joint_jerk_index = joint;
    }
  }
  consider(diagnostics.max_joint_velocity_ratio,
           TimeParameterConstraint::JOINT_VELOCITY_SLOPE);
  consider(diagnostics.max_joint_acceleration_ratio,
           TimeParameterConstraint::JOINT_ACCELERATION);
  consider(diagnostics.max_joint_jerk_ratio,
           TimeParameterConstraint::JOINT_JERK);

  diagnostics.linear_velocity_ratio =
      utilization(result.max_linear_velocity_m_s,
                  config.limits.max_linear_velocity_m_s * config.linear_scale);
  diagnostics.linear_acceleration_ratio = utilization(
      result.max_linear_acceleration_m_s2,
      config.limits.max_linear_acceleration_m_s2 *
          std::sqrt(config.linear_scale));
  diagnostics.linear_jerk_ratio =
      utilization(result.max_linear_jerk_m_s3,
                  config.limits.max_linear_jerk_m_s3 * config.linear_scale);
  diagnostics.angular_velocity_ratio = utilization(
      result.max_angular_velocity_rad_s,
      config.limits.max_angular_velocity_rad_s * config.angular_scale);
  diagnostics.angular_acceleration_ratio = utilization(
      result.max_angular_acceleration_rad_s2,
      config.limits.max_angular_acceleration_rad_s2 *
          std::sqrt(config.angular_scale));
  diagnostics.angular_jerk_ratio =
      utilization(result.max_angular_jerk_rad_s3,
                  config.limits.max_angular_jerk_rad_s3 * config.angular_scale);
  consider(diagnostics.linear_velocity_ratio,
           TimeParameterConstraint::LINEAR_VELOCITY_DERIVATIVE);
  consider(diagnostics.linear_acceleration_ratio,
           TimeParameterConstraint::LINEAR_ACCELERATION);
  consider(diagnostics.linear_jerk_ratio, TimeParameterConstraint::LINEAR_JERK);
  consider(diagnostics.angular_velocity_ratio,
           TimeParameterConstraint::ANGULAR_VELOCITY_DERIVATIVE);
  consider(diagnostics.angular_acceleration_ratio,
           TimeParameterConstraint::ANGULAR_ACCELERATION);
  consider(diagnostics.angular_jerk_ratio,
           TimeParameterConstraint::ANGULAR_JERK);
}

bool metricsWithinLimits(PathTimeParameterizationResult &result,
                         const PathTimeParameterizerConfig &config) {
  constexpr double kNumericalSlack = 1e-7;
  populateDiagnostics(result, config);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (result.max_joint_velocity_by_joint_rad_s[joint] >
            config.limits.max_joint_velocity_rad_s[joint] *
                    config.joint_velocity_scale +
                kNumericalSlack ||
        result.max_joint_acceleration_by_joint_rad_s2[joint] >
            config.limits.max_joint_acceleration_rad_s2[joint] *
                    config.joint_acceleration_scale +
                kNumericalSlack ||
        result.max_joint_jerk_by_joint_rad_s3[joint] >
            config.limits.max_joint_jerk_rad_s3[joint] *
                    config.joint_jerk_scale +
                kNumericalSlack) {
      return false;
    }
  }
  return result.max_linear_velocity_m_s <=
             config.limits.max_linear_velocity_m_s * config.linear_scale +
                 kNumericalSlack &&
         result.max_linear_acceleration_m_s2 <=
             config.limits.max_linear_acceleration_m_s2 *
                     std::sqrt(config.linear_scale) +
                 kNumericalSlack &&
         result.max_linear_jerk_m_s3 <=
             config.limits.max_linear_jerk_m_s3 * config.linear_scale +
                 kNumericalSlack &&
         result.max_angular_velocity_rad_s <=
             config.limits.max_angular_velocity_rad_s * config.angular_scale +
                 kNumericalSlack &&
         result.max_angular_acceleration_rad_s2 <=
             config.limits.max_angular_acceleration_rad_s2 *
                     std::sqrt(config.angular_scale) +
                 kNumericalSlack &&
         result.max_angular_jerk_rad_s3 <=
             config.limits.max_angular_jerk_rad_s3 * config.angular_scale +
                 kNumericalSlack;
}

