#include "fr10_motion_core/cartesian_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "fr10_motion_core/math_utils.hpp"
#include "fr10_motion_core/pre_insert_planner.hpp"

namespace fr10_motion {
namespace {

bool finitePose(const Pose &pose) {
  return std::all_of(pose.position_m.begin(), pose.position_m.end(),
                     [](double value) { return std::isfinite(value); }) &&
         std::all_of(pose.orientation_quat.begin(),
                     pose.orientation_quat.end(),
                     [](double value) { return std::isfinite(value); }) &&
         math::quaternionNormSquared(pose.orientation_quat) > 1e-12;
}

void setError(std::string *error, const char *message) {
  if (error != nullptr) {
    *error = message;
  }
}

} // namespace

Pose CartesianPath::interpolate(const Pose &start, const Pose &goal,
                                double fraction) {
  fraction = std::clamp(fraction, 0.0, 1.0);
  Pose result;
  for (std::size_t i = 0; i < result.position_m.size(); ++i) {
    result.position_m[i] =
        start.position_m[i] + fraction * (goal.position_m[i] - start.position_m[i]);
  }
  result.orientation_quat = math::quaternionSlerp(
      start.orientation_quat, goal.orientation_quat, fraction);
  return result;
}

Pose CartesianPath::interpolate(const std::vector<Pose> &waypoints,
                                double fraction) {
  if (waypoints.empty()) {
    return Pose{};
  }
  if (waypoints.size() == 1) {
    return waypoints.front();
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double scaled =
      fraction * static_cast<double>(waypoints.size() - 1);
  const std::size_t segment = std::min<std::size_t>(
      static_cast<std::size_t>(scaled), waypoints.size() - 2);
  const double local = std::clamp(
      scaled - static_cast<double>(segment), 0.0, 1.0);
  return interpolate(waypoints[segment], waypoints[segment + 1], local);
}

std::array<double, 3> CartesianPath::translationTangent(
    const std::vector<Pose> &waypoints, double fraction) {
  std::array<double, 3> tangent{};
  if (waypoints.size() < 2) {
    return tangent;
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double scaled =
      fraction * static_cast<double>(waypoints.size() - 1);
  const std::size_t segment = std::min<std::size_t>(
      static_cast<std::size_t>(scaled), waypoints.size() - 2);
  double squared_norm = 0.0;
  for (std::size_t axis = 0; axis < tangent.size(); ++axis) {
    tangent[axis] = waypoints[segment + 1].position_m[axis] -
                    waypoints[segment].position_m[axis];
    squared_norm += tangent[axis] * tangent[axis];
  }
  const double norm = std::sqrt(squared_norm);
  if (norm > std::numeric_limits<double>::epsilon()) {
    for (double &value : tangent) {
      value /= norm;
    }
  }
  return tangent;
}

double CartesianPath::translationDeviation(
    const Pose &reference, const std::array<double, 3> &unit_tangent,
    const Pose &sample) {
  double tangent_norm_squared = 0.0;
  double error_norm_squared = 0.0;
  double projection = 0.0;
  for (std::size_t axis = 0; axis < unit_tangent.size(); ++axis) {
    const double error =
        sample.position_m[axis] - reference.position_m[axis];
    tangent_norm_squared += unit_tangent[axis] * unit_tangent[axis];
    error_norm_squared += error * error;
    projection += error * unit_tangent[axis];
  }
  if (tangent_norm_squared <= std::numeric_limits<double>::epsilon()) {
    return std::sqrt(error_norm_squared);
  }
  return std::sqrt(std::max(
      0.0, error_norm_squared - projection * projection /
                                    tangent_norm_squared));
}

bool CartesianPath::linearSegmentCount(
    const Pose &start, const Pose &goal, const CartesianPathConfig &config,
    std::size_t &segments, std::string *error) {
  segments = 0;
  if (!finitePose(start) || !finitePose(goal)) {
    setError(error, "path endpoints must be finite poses with valid quaternions");
    return false;
  }
  if (!(config.max_translation_step_m > 0.0) ||
      !std::isfinite(config.max_translation_step_m) ||
      !(config.max_rotation_step_rad > 0.0) ||
      !std::isfinite(config.max_rotation_step_rad) ||
      config.max_waypoints < 2) {
    setError(error, "Cartesian path configuration is invalid");
    return false;
  }

  double distance_squared = 0.0;
  for (std::size_t i = 0; i < start.position_m.size(); ++i) {
    const double delta = goal.position_m[i] - start.position_m[i];
    distance_squared += delta * delta;
  }
  const double distance = std::sqrt(distance_squared);
  const double angular_distance = math::quaternionAngularDistance(
      start.orientation_quat, goal.orientation_quat);
  if (!std::isfinite(distance) || !std::isfinite(angular_distance)) {
    setError(error, "Cartesian path distance or orientation is invalid");
    return false;
  }

  const double translation_segments =
      std::ceil(distance / config.max_translation_step_m);
  const double rotation_segments =
      std::ceil(angular_distance / config.max_rotation_step_rad);
  const double required_segments =
      std::max(1.0, std::max(translation_segments, rotation_segments));
  if (!std::isfinite(required_segments) ||
      required_segments >=
          static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
      required_segments > static_cast<double>(config.max_waypoints - 1)) {
    setError(error, "Cartesian path exceeds configured waypoint capacity");
    return false;
  }
  segments = static_cast<std::size_t>(required_segments);
  return true;
}

bool CartesianPath::linear(const Pose &start, const Pose &goal,
                           const CartesianPathConfig &config,
                           std::vector<Pose> &waypoints,
                           std::string *error) {
  waypoints.clear();
  std::size_t segments = 0;
  if (!linearSegmentCount(start, goal, config, segments, error)) {
    return false;
  }
  waypoints.reserve(segments + 1);
  for (std::size_t index = 0; index <= segments; ++index) {
    waypoints.push_back(interpolate(
        start, goal, static_cast<double>(index) / static_cast<double>(segments)));
  }
  return true;
}

bool CartesianPath::alternateOrientation(
    const Pose &start, const Pose &goal, const CartesianPathConfig &config,
    std::vector<Pose> &waypoints, std::string *error) {
  waypoints.clear();
  if (!finitePose(start) || !finitePose(goal) ||
      !(config.max_translation_step_m > 0.0) ||
      !std::isfinite(config.max_translation_step_m) ||
      !(config.max_rotation_step_rad > 0.0) ||
      !std::isfinite(config.max_rotation_step_rad) ||
      config.max_waypoints < 2) {
    setError(error,
             "alternate-orientation path input/configuration is invalid");
    return false;
  }
  double distance_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = goal.position_m[axis] - start.position_m[axis];
    distance_squared += delta * delta;
  }
  const double shortest_angle = math::quaternionAngularDistance(
      start.orientation_quat, goal.orientation_quat);
  if (!std::isfinite(shortest_angle) || shortest_angle < 1e-6 ||
      shortest_angle > 2.0 * std::acos(-1.0) - 1e-6) {
    setError(error,
             "alternate orientation arc is undefined for this endpoint pair");
    return false;
  }
  const double alternate_angle = 2.0 * std::acos(-1.0) - shortest_angle;
  const std::size_t translation_segments = static_cast<std::size_t>(
      std::ceil(std::sqrt(distance_squared) /
                config.max_translation_step_m));
  const std::size_t rotation_segments = static_cast<std::size_t>(
      std::ceil(alternate_angle / config.max_rotation_step_rad));
  const std::size_t segments =
      std::max<std::size_t>(1, std::max(translation_segments,
                                       rotation_segments));
  if (segments + 1 > config.max_waypoints) {
    setError(error,
             "alternate-orientation path exceeds configured waypoint capacity");
    return false;
  }
  waypoints.reserve(segments + 1);
  for (std::size_t index = 0; index <= segments; ++index) {
    const double fraction = static_cast<double>(index) /
                            static_cast<double>(segments);
    Pose pose = interpolate(start, goal, fraction);
    pose.orientation_quat = math::quaternionSlerpLong(
        start.orientation_quat, goal.orientation_quat, fraction);
    waypoints.push_back(pose);
  }
  waypoints.front() = start;
  waypoints.back().position_m = goal.position_m;
  // Keep the long-arc quaternion sign. q and -q represent the same endpoint.
  waypoints.back().orientation_quat = math::quaternionSlerpLong(
      start.orientation_quat, goal.orientation_quat, 1.0);
  return true;
}

bool CartesianPath::detour(const Pose &start, const Pose &goal,
                           double peak_fraction,
                           const std::array<double, 3> &peak_offset_m,
                           const CartesianPathConfig &config,
                           std::vector<Pose> &waypoints,
                           std::string *error) {
  waypoints.clear();
  if (!finitePose(start) || !finitePose(goal) ||
      !(peak_fraction > 0.0 && peak_fraction < 1.0) ||
      !std::isfinite(peak_fraction) ||
      !std::all_of(peak_offset_m.begin(), peak_offset_m.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !(config.max_translation_step_m > 0.0) ||
      !std::isfinite(config.max_translation_step_m) ||
      !(config.max_rotation_step_rad > 0.0) ||
      !std::isfinite(config.max_rotation_step_rad) ||
      config.max_waypoints < 2) {
    setError(error, "Cartesian detour input/configuration is invalid");
    return false;
  }

  double direct_distance_squared = 0.0;
  double offset_squared = 0.0;
  for (std::size_t axis = 0; axis < peak_offset_m.size(); ++axis) {
    const double delta = goal.position_m[axis] - start.position_m[axis];
    direct_distance_squared += delta * delta;
    offset_squared += peak_offset_m[axis] * peak_offset_m[axis];
  }
  const double direct_distance = std::sqrt(direct_distance_squared);
  const double offset_norm = std::sqrt(offset_squared);
  if (!(offset_norm > std::numeric_limits<double>::epsilon())) {
    setError(error, "Cartesian detour offset must be non-zero");
    return false;
  }
  const double angular_distance = math::quaternionAngularDistance(
      start.orientation_quat, goal.orientation_quat);
  if (!std::isfinite(angular_distance)) {
    setError(error, "Cartesian detour orientation is invalid");
    return false;
  }

  std::size_t segments = std::max<std::size_t>(
      1, std::max(
             static_cast<std::size_t>(std::ceil(
                 (direct_distance + 2.0 * offset_norm) /
                 config.max_translation_step_m)),
             static_cast<std::size_t>(std::ceil(
                 angular_distance / config.max_rotation_step_rad))));

  const auto create = [&](std::size_t segment_count) {
    std::vector<Pose> candidate;
    candidate.reserve(segment_count + 1);
    for (std::size_t index = 0; index <= segment_count; ++index) {
      const double fraction = static_cast<double>(index) /
                              static_cast<double>(segment_count);
      Pose pose = interpolate(start, goal, fraction);
      const double bump = fraction <= peak_fraction
                              ? std::sin(0.5 * std::acos(-1.0) * fraction /
                                         peak_fraction)
                              : std::sin(0.5 * std::acos(-1.0) *
                                         (1.0 - fraction) /
                                         (1.0 - peak_fraction));
      for (std::size_t axis = 0; axis < peak_offset_m.size(); ++axis) {
        pose.position_m[axis] += bump * peak_offset_m[axis];
      }
      candidate.push_back(pose);
    }
    candidate.front() = start;
    candidate.back() = goal;
    return candidate;
  };

  while (segments + 1 <= config.max_waypoints) {
    auto candidate = create(segments);
    double max_translation_step = 0.0;
    double max_rotation_step = 0.0;
    for (std::size_t index = 1; index < candidate.size(); ++index) {
      double step_squared = 0.0;
      for (std::size_t axis = 0; axis < peak_offset_m.size(); ++axis) {
        const double delta = candidate[index].position_m[axis] -
                             candidate[index - 1].position_m[axis];
        step_squared += delta * delta;
      }
      max_translation_step =
          std::max(max_translation_step, std::sqrt(step_squared));
      max_rotation_step = std::max(
          max_rotation_step,
          math::quaternionAngularDistance(
              candidate[index - 1].orientation_quat,
              candidate[index].orientation_quat));
    }
    const double ratio = std::max(
        max_translation_step / config.max_translation_step_m,
        max_rotation_step / config.max_rotation_step_rad);
    if (ratio <= 1.0 + 1e-10) {
      waypoints = std::move(candidate);
      return true;
    }
    const std::size_t next_segments = static_cast<std::size_t>(
        std::ceil(static_cast<double>(segments) * ratio * 1.01));
    segments = std::max(segments + 1, next_segments);
  }

  setError(error, "Cartesian detour exceeds configured waypoint capacity");
  return false;
}

bool CartesianPath::stagedDetour(
    const Pose &start, const Pose &goal, double staging_fraction,
    const std::array<double, 3> &staging_offset_m,
    const CartesianPathConfig &config, std::vector<Pose> &waypoints,
    std::string *error) {
  waypoints.clear();
  if (!finitePose(start) || !finitePose(goal) ||
      !(staging_fraction > 0.0 && staging_fraction < 1.0) ||
      !std::isfinite(staging_fraction) ||
      !std::all_of(staging_offset_m.begin(), staging_offset_m.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !(config.max_translation_step_m > 0.0) ||
      !std::isfinite(config.max_translation_step_m) ||
      !(config.max_rotation_step_rad > 0.0) ||
      !std::isfinite(config.max_rotation_step_rad) ||
      config.max_waypoints < 2) {
    setError(error, "staged Cartesian detour input/configuration is invalid");
    return false;
  }

  Pose staging = interpolate(start, goal, staging_fraction);
  staging.orientation_quat = start.orientation_quat;
  double offset_squared = 0.0;
  for (std::size_t axis = 0; axis < staging_offset_m.size(); ++axis) {
    staging.position_m[axis] += staging_offset_m[axis];
    offset_squared += staging_offset_m[axis] * staging_offset_m[axis];
  }
  if (!(offset_squared > std::numeric_limits<double>::epsilon())) {
    setError(error, "staged Cartesian detour offset must be non-zero");
    return false;
  }

  const auto positionDistance = [](const Pose &first, const Pose &second) {
    double squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = second.position_m[axis] - first.position_m[axis];
      squared += delta * delta;
    }
    return std::sqrt(squared);
  };
  const double translation_length =
      positionDistance(start, staging) + positionDistance(staging, goal);
  const double angular_distance = math::quaternionAngularDistance(
      start.orientation_quat, goal.orientation_quat);
  if (!std::isfinite(angular_distance)) {
    setError(error, "staged Cartesian detour orientation is invalid");
    return false;
  }
  std::size_t segments = std::max<std::size_t>(
      3, std::max(
             static_cast<std::size_t>(std::ceil(
                 translation_length / config.max_translation_step_m)),
             static_cast<std::size_t>(std::ceil(
                 angular_distance / config.max_rotation_step_rad))));

  const auto smoothstep = [](double value) {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * value *
           (10.0 + value * (-15.0 + 6.0 * value));
  };
  const auto create = [&](std::size_t segment_count) {
    std::vector<Pose> candidate;
    candidate.reserve(segment_count + 1);
    for (std::size_t index = 0; index <= segment_count; ++index) {
      const double fraction = static_cast<double>(index) /
                              static_cast<double>(segment_count);
      Pose pose;
      if (fraction < 1.0 / 3.0) {
        const double phase = smoothstep(3.0 * fraction);
        pose = interpolate(start, staging, phase);
        pose.orientation_quat = start.orientation_quat;
      } else if (fraction < 2.0 / 3.0) {
        const double phase = smoothstep(3.0 * fraction - 1.0);
        pose = staging;
        pose.orientation_quat = math::quaternionSlerp(
            start.orientation_quat, goal.orientation_quat, phase);
      } else {
        const double phase = smoothstep(3.0 * fraction - 2.0);
        Pose staging_with_goal_orientation = staging;
        staging_with_goal_orientation.orientation_quat =
            goal.orientation_quat;
        pose = interpolate(staging_with_goal_orientation, goal, phase);
        pose.orientation_quat = goal.orientation_quat;
      }
      candidate.push_back(pose);
    }
    candidate.front() = start;
    candidate.back() = goal;
    return candidate;
  };

  while (segments + 1 <= config.max_waypoints) {
    auto candidate = create(segments);
    double max_translation_step = 0.0;
    double max_rotation_step = 0.0;
    for (std::size_t index = 1; index < candidate.size(); ++index) {
      max_translation_step = std::max(
          max_translation_step,
          positionDistance(candidate[index - 1], candidate[index]));
      max_rotation_step = std::max(
          max_rotation_step,
          math::quaternionAngularDistance(
              candidate[index - 1].orientation_quat,
              candidate[index].orientation_quat));
    }
    const double ratio = std::max(
        max_translation_step / config.max_translation_step_m,
        max_rotation_step / config.max_rotation_step_rad);
    if (ratio <= 1.0 + 1e-10) {
      waypoints = std::move(candidate);
      return true;
    }
    const std::size_t next_segments = static_cast<std::size_t>(
        std::ceil(static_cast<double>(segments) * ratio * 1.01));
    segments = std::max(segments + 1, next_segments);
  }

  setError(error,
           "staged Cartesian detour exceeds configured waypoint capacity");
  return false;
}

double CartesianPath::closestTranslationParameter(const Pose &start,
                                                   const Pose &goal,
                                                   const Pose &sample) {
  double length_squared = 0.0;
  double projection = 0.0;
  for (std::size_t i = 0; i < start.position_m.size(); ++i) {
    const double direction = goal.position_m[i] - start.position_m[i];
    length_squared += direction * direction;
    projection += (sample.position_m[i] - start.position_m[i]) * direction;
  }
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return std::clamp(projection / length_squared, 0.0, 1.0);
}

double CartesianPath::translationDeviation(const Pose &start, const Pose &goal,
                                           const Pose &sample) {
  const Pose closest =
      interpolate(start, goal, closestTranslationParameter(start, goal, sample));
  double error_squared = 0.0;
  for (std::size_t i = 0; i < sample.position_m.size(); ++i) {
    const double error = sample.position_m[i] - closest.position_m[i];
    error_squared += error * error;
  }
  return std::sqrt(error_squared);
}

bool CartesianPath::preInsertPose(
    const Pose &target, const std::array<double, 3> &local_axis,
    double distance_m, Pose &pre_insert, std::string *error) {
  PreInsertPlannerConfig config;
  config.local_axis = local_axis;
  config.distance_m = distance_m;
  config.min_distance_m = 0.0;
  config.max_distance_m = std::numeric_limits<double>::max();
  const auto plan = PreInsertPlanner::plan(target, config);
  if (!plan.success) {
    if (error != nullptr) {
      *error = plan.message;
    }
    return false;
  }
  pre_insert = plan.pre_insert_pose;
  return true;
}

} // namespace fr10_motion
