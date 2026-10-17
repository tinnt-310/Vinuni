#include "fr10_motion_core/cartesian_route_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {
namespace {

double norm(const std::array<double, 3> &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

std::vector<std::array<double, 3>> detourDirections(const Pose &start,
                                                    const Pose &goal) {
  std::vector<std::array<double, 3>> preferred{
      {{0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0}},  {{0.0, -1.0, 0.0}},
      {{1.0, 0.0, 0.0}}, {{-1.0, 0.0, 0.0}}, {{0.0, 0.0, -1.0}}};
  for (int z = 1; z >= -1; --z) {
    for (int y = 1; y >= -1; --y) {
      for (int x = 1; x >= -1; --x) {
        const int non_zero =
            (x != 0 ? 1 : 0) + (y != 0 ? 1 : 0) + (z != 0 ? 1 : 0);
        if (non_zero >= 2) {
          preferred.push_back({static_cast<double>(x), static_cast<double>(y),
                               static_cast<double>(z)});
        }
      }
    }
  }
  std::array<double, 3> line{};
  for (std::size_t axis = 0; axis < line.size(); ++axis) {
    line[axis] = goal.position_m[axis] - start.position_m[axis];
  }
  const double line_norm = norm(line);
  if (line_norm > std::numeric_limits<double>::epsilon()) {
    for (double &value : line) {
      value /= line_norm;
    }
  }

  std::vector<std::array<double, 3>> directions;
  directions.reserve(preferred.size());
  for (auto candidate : preferred) {
    if (line_norm > std::numeric_limits<double>::epsilon()) {
      double projection = 0.0;
      for (std::size_t axis = 0; axis < line.size(); ++axis) {
        projection += candidate[axis] * line[axis];
      }
      for (std::size_t axis = 0; axis < line.size(); ++axis) {
        candidate[axis] -= projection * line[axis];
      }
    }
    const double candidate_norm = norm(candidate);
    if (candidate_norm <= 1e-9) {
      continue;
    }
    for (double &value : candidate) {
      value /= candidate_norm;
    }
    bool duplicate = false;
    for (const auto &existing : directions) {
      double dot = 0.0;
      for (std::size_t axis = 0; axis < candidate.size(); ++axis) {
        dot += candidate[axis] * existing[axis];
      }
      duplicate = duplicate || dot > 1.0 - 1e-9;
    }
    if (!duplicate) {
      directions.push_back(candidate);
    }
  }
  return directions;
}

bool validConfig(const CartesianRoutePlannerConfig &config) {
  bool valid_joint_bounds = true;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    valid_joint_bounds =
        valid_joint_bounds &&
        std::isfinite(config.limits.min_joint_position_rad[joint]) &&
        std::isfinite(config.limits.max_joint_position_rad[joint]) &&
        config.limits.min_joint_position_rad[joint] <
            config.limits.max_joint_position_rad[joint];
  }
  return valid_joint_bounds && config.path.max_translation_step_m > 0.0 &&
         std::isfinite(config.path.max_translation_step_m) &&
         config.path.max_rotation_step_rad > 0.0 &&
         std::isfinite(config.path.max_rotation_step_rad) &&
         config.path.max_waypoints >= 2 &&
         config.continuous_ik.max_joint_step_rad > 0.0 &&
         config.joint_space_step_rad > 0.0 &&
         std::isfinite(config.joint_space_step_rad) &&
         config.cylindrical_barrier_radius_m > 0.0 &&
         std::isfinite(config.cylindrical_barrier_radius_m) &&
         config.cylindrical_detour_radius_m >=
             config.cylindrical_barrier_radius_m &&
         std::isfinite(config.cylindrical_detour_radius_m) &&
         std::all_of(config.detour_clearances_m.begin(),
                     config.detour_clearances_m.end(), [](double value) {
                       return value > 0.0 && std::isfinite(value);
                     });
}

double planarSegmentMinimumRadius(const Pose &start, const Pose &goal) {
  const double dx = goal.position_m[0] - start.position_m[0];
  const double dy = goal.position_m[1] - start.position_m[1];
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(start.position_m[0], start.position_m[1]);
  }
  const double projection = std::clamp(
      -(start.position_m[0] * dx + start.position_m[1] * dy) /
          length_squared,
      0.0, 1.0);
  return std::hypot(start.position_m[0] + projection * dx,
                    start.position_m[1] + projection * dy);
}

KinematicState interpolateState(const KinematicState &start,
                                const KinematicState &goal, double fraction) {
  KinematicState state;
  fraction = std::clamp(fraction, 0.0, 1.0);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    state.position_rad[joint] =
        start.position_rad[joint] +
        fraction * (goal.position_rad[joint] - start.position_rad[joint]);
  }
  return state;
}

double quinticSmoothStep(double value) {
  value = std::clamp(value, 0.0, 1.0);
  return value * value * value *
         (10.0 + value * (-15.0 + 6.0 * value));
}

double cylindricalRadius(double fraction, double start_radius,
                         double goal_radius, double detour_radius) {
  if (fraction <= 0.5) {
    const double phase = quinticSmoothStep(2.0 * fraction);
    return start_radius + phase * (detour_radius - start_radius);
  }
  const double phase = quinticSmoothStep(2.0 * fraction - 1.0);
  return detour_radius + phase * (goal_radius - detour_radius);
}

bool cylindricalDetour(const Pose &start, const Pose &goal,
                       const KinematicState &seed, IKProvider &provider,
                       const CartesianRoutePlannerConfig &config,
                       CartesianRoutePlan &result,
                       const CartesianRoutePlanner::CancelCheck &cancel) {
  constexpr double kPi = 3.141592653589793238462643383279502884;
  constexpr double kTwoPi = 2.0 * kPi;
  const double start_radius = std::hypot(start.position_m[0], start.position_m[1]);
  const double goal_radius = std::hypot(goal.position_m[0], goal.position_m[1]);
  if (start_radius < config.cylindrical_barrier_radius_m ||
      goal_radius < config.cylindrical_barrier_radius_m) {
    return false;
  }
  const double start_angle =
      std::atan2(start.position_m[1], start.position_m[0]);
  const double goal_angle =
      std::atan2(goal.position_m[1], goal.position_m[0]);
  const double shortest_delta = std::remainder(goal_angle - start_angle,
                                                kTwoPi);
  const std::array<double, 2> angle_deltas{
      shortest_delta,
      shortest_delta >= 0.0 ? shortest_delta - kTwoPi
                            : shortest_delta + kTwoPi};

  for (const double angle_delta : angle_deltas) {
    const double translation_bound =
        std::abs(start_radius - config.cylindrical_detour_radius_m) +
        config.cylindrical_detour_radius_m * std::abs(angle_delta) +
        std::abs(goal_radius - config.cylindrical_detour_radius_m) +
        std::abs(goal.position_m[2] - start.position_m[2]);
    const double rotation = math::quaternionAngularDistance(
        start.orientation_quat, goal.orientation_quat);
    const std::size_t segments = std::max<std::size_t>(
        2, std::max(
               static_cast<std::size_t>(std::ceil(
                   translation_bound / config.path.max_translation_step_m)),
               static_cast<std::size_t>(std::ceil(
                   rotation / config.path.max_rotation_step_rad))));
    if (segments + 1 > config.path.max_waypoints) {
      continue;
    }
    std::vector<Pose> path;
    path.reserve(segments + 1);
    for (std::size_t index = 0; index <= segments; ++index) {
      if (cancel && cancel()) {
        return false;
      }
      const double fraction =
          static_cast<double>(index) / static_cast<double>(segments);
      const double blend = quinticSmoothStep(fraction);
      Pose pose = CartesianPath::interpolate(start, goal, blend);
      const double radius = cylindricalRadius(
          fraction, start_radius, goal_radius,
          config.cylindrical_detour_radius_m);
      const double angle = start_angle + blend * angle_delta;
      pose.position_m[0] = radius * std::cos(angle);
      pose.position_m[1] = radius * std::sin(angle);
      path.push_back(pose);
    }
    auto continuous = ContinuousIK::solve(path, seed, provider,
                                          config.continuous_ik, cancel);
    if (!continuous.success) {
      continue;
    }
    result.success = true;
    result.used_detour = true;
    result.used_joint_space_detour = true;
    result.used_cylindrical_detour = true;
    result.cylindrical_detour_waypoints = path.size();
    result.detour_clearance_m = config.cylindrical_detour_radius_m;
    result.cartesian_path = std::move(path);
    result.continuous_ik = std::move(continuous);
    result.message = "success: deterministic cylindrical detour";
    return true;
  }
  return false;
}

// Uniform six-joint arc sampling prevents unequal edge lengths from creating
// artificial derivative jumps in the timing spline. Every sampled state and
// the eventual timed curve still receive authoritative FK/collision checks.
std::vector<KinematicState> sampleJointPolyline(
    const std::vector<KinematicState> &points, double step, std::size_t capacity) {
  std::vector<double> distance(points.size(),0.0);
  for(std::size_t i=1;i<points.size();++i) {
    double squared=0.0;
    for(std::size_t j=0;j<kJointCount;++j) {
      const double delta=points[i].position_rad[j]-points[i-1].position_rad[j];
      squared+=delta*delta;
    }
    distance[i]=distance[i-1]+std::sqrt(squared);
  }
  const auto count=std::max<std::size_t>(1,std::ceil(distance.back()/step));
  if(count+1>capacity)return {};
  std::vector<KinematicState> sampled;sampled.reserve(count+1);
  std::size_t edge=1;
  for(std::size_t i=0;i<=count;++i) {
    const double at=distance.back()*i/count;
    while(edge+1<distance.size() && distance[edge]<at)++edge;
    const double length=distance[edge]-distance[edge-1];
    sampled.push_back(interpolateState(points[edge-1],points[edge],
        length>1e-15?(at-distance[edge-1])/length:0.0));
  }
  sampled.front()=points.front();sampled.back()=points.back();
  return sampled;
}

std::vector<KinematicState> blendJointCorners(
    const std::vector<KinematicState> &points, double radius) {
  std::vector<KinematicState> blended{points.front()};
  for(std::size_t i=1;i+1<points.size();++i) {
    double incoming=0.0,outgoing=0.0;
    for(std::size_t j=0;j<kJointCount;++j) {
      incoming=std::hypot(incoming,points[i].position_rad[j]-points[i-1].position_rad[j]);
      outgoing=std::hypot(outgoing,points[i+1].position_rad[j]-points[i].position_rad[j]);
    }
    if(incoming<=1e-12 || outgoing<=1e-12)continue;
    const double trim=std::min({radius,.25*incoming,.25*outgoing});
    const auto before=interpolateState(points[i-1],points[i],1.0-trim/incoming);
    const auto after=interpolateState(points[i],points[i+1],trim/outgoing);
    blended.push_back(before);
    // Quadratic fillet stays in the convex hull of the three waypoints.
    // Clearance must still be validated; a colliding blend is discarded.
    for(unsigned sample=1;sample<=32;++sample) {
      const double t=sample/32.0;
      KinematicState q;
      for(std::size_t j=0;j<kJointCount;++j)
        q.position_rad[j]=(1-t)*(1-t)*before.position_rad[j]+
            2*t*(1-t)*points[i].position_rad[j]+t*t*after.position_rad[j];
      blended.push_back(q);
    }
  }
  blended.push_back(points.back());return blended;
}

bool materializeJointRoute(const std::vector<KinematicState> &coarse_route,
                           const Pose &goal, IKProvider &provider,
                           const CartesianRoutePlannerConfig &config,
                           CartesianRoutePlan &result,
                           const CartesianRoutePlanner::CancelCheck &cancel) {
  if (coarse_route.size() < 2) {
    return false;
  }
  double effective_step = config.joint_space_step_rad;
  for (std::size_t density_iteration = 0; density_iteration < 8;
       ++density_iteration) {
    const auto sampled=sampleJointPolyline(coarse_route,effective_step,config.path.max_waypoints);
    if(sampled.empty()) {
      result.direct_failure_message="joint route exceeds configured waypoint count";
      return false;
    }
    const auto waypoint_count=sampled.size();

    std::vector<KinematicState> joint_path;
    std::vector<Pose> cartesian_path;
    joint_path.reserve(waypoint_count);
    cartesian_path.reserve(waypoint_count);
    ContinuousIKResult metrics;
    metrics.min_manipulability = std::numeric_limits<double>::infinity();
    double required_density_ratio = 1.0;
    KinematicState previous = coarse_route.front();
    for (const auto &sample : sampled) {
        if (cancel && cancel()) return false;
        KinematicState state=sample;
        IKResult validation;
        if (joint_path.empty()) {
          state = coarse_route.front();
          validation.success = true;
          validation.manipulability = std::numeric_limits<double>::infinity();
        } else {
          validation = provider.validate(state, previous);
        }
        Pose pose;
        if (!validation.success) {
          result.direct_failure_index = joint_path.size();
          result.direct_failure_message =
              "joint route validation at sample " +
              std::to_string(joint_path.size()) + ": " + validation.message;
          return false;
        }
        if (!provider.forward(state, pose, &metrics.message)) {
          result.direct_failure_index = joint_path.size();
          result.direct_failure_message =
              "joint route FK at sample " +
              std::to_string(joint_path.size()) + ": " + metrics.message;
          return false;
        }
        if (!cartesian_path.empty()) {
          double translation_squared = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double delta =
                pose.position_m[axis] - cartesian_path.back().position_m[axis];
            translation_squared += delta * delta;
          }
          const double rotation_step = math::quaternionAngularDistance(
              cartesian_path.back().orientation_quat, pose.orientation_quat);
          required_density_ratio = std::max(
              required_density_ratio,
              std::max(std::sqrt(translation_squared) /
                           config.path.max_translation_step_m,
                       rotation_step / config.path.max_rotation_step_rad));
        }
        metrics.min_manipulability =
            std::min(metrics.min_manipulability, validation.manipulability);
        metrics.max_condition_number =
            std::max(metrics.max_condition_number, validation.condition_number);
        joint_path.push_back(state);
        cartesian_path.push_back(pose);
        previous = state;
    }
    if (required_density_ratio > 1.0 + 1e-10) {
      effective_step /= required_density_ratio * 1.01;
      continue;
    }

    double endpoint_position_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          cartesian_path.back().position_m[axis] - goal.position_m[axis];
      endpoint_position_squared += delta * delta;
    }
    const double endpoint_position_error = std::sqrt(endpoint_position_squared);
    const double endpoint_orientation_error = math::quaternionAngularDistance(
        cartesian_path.back().orientation_quat, goal.orientation_quat);
    if (endpoint_position_error > config.continuous_ik.max_position_error_m ||
        endpoint_orientation_error >
            config.continuous_ik.max_orientation_error_rad) {
      result.direct_failure_message =
          "joint route endpoint Cartesian error exceeds configured limit";
      return false;
    }
    metrics.success = true;
    metrics.message = "success";
    metrics.joint_path = std::move(joint_path);
    metrics.max_position_error_m = endpoint_position_error;
    metrics.max_orientation_error_rad = endpoint_orientation_error;
    if (!std::isfinite(metrics.min_manipulability)) {
      metrics.min_manipulability = 0.0;
    }
    result.success = true;
    result.used_detour = true;
    result.used_joint_space_detour = true;
    result.cartesian_path = std::move(cartesian_path);
    result.continuous_ik = std::move(metrics);
    result.message = "success: validated joint-space kinematic detour";
    return true;
  }
  return false;
}

bool jointSpaceDetour(const Pose &start, const Pose &goal,
                      const KinematicState &seed,
                      IKProvider &provider,
                      const CartesianRoutePlannerConfig &config,
                      CartesianRoutePlan &result,
                      const CartesianRoutePlanner::CancelCheck &cancel) {
  std::array<IKResult, IKProvider::kMaxCandidates> endpoints{};
  std::string endpoint_error;
  const std::size_t endpoint_count =
      provider.solveCandidates(goal, seed, endpoints, &endpoint_error);
  std::array<std::pair<double, std::size_t>, IKProvider::kMaxCandidates>
      ranked{};
  for (std::size_t endpoint = 0; endpoint < endpoint_count; ++endpoint) {
    double cost = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double delta = endpoints[endpoint].state.position_rad[joint] -
                           seed.position_rad[joint];
      cost += delta * delta;
    }
    ranked[endpoint] = {cost, endpoint};
  }
  std::sort(ranked.begin(),
            ranked.begin() + static_cast<std::ptrdiff_t>(endpoint_count));

  for (std::size_t rank = 0; rank < endpoint_count; ++rank) {
    const KinematicState &endpoint = endpoints[ranked[rank].second].state;
    std::vector<KinematicState> coarse_route{seed, endpoint};
    if (materializeJointRoute(coarse_route, goal, provider, config, result,
                              cancel)) {
      return true;
    }
  }
  if (planarSegmentMinimumRadius(start, goal) <
      config.cylindrical_barrier_radius_m) {
    return cylindricalDetour(start, goal, seed, provider, config, result,
                             cancel);
  }
  return false;
}

} // namespace

CartesianRoutePlan CartesianRoutePlanner::fromJointWaypoints(
    const std::vector<KinematicState> &waypoints, const Pose &goal,
    IKProvider &provider, const CartesianRoutePlannerConfig &config,
    const CancelCheck &cancel) {
  CartesianRoutePlan result;
  if (!validConfig(config) || waypoints.size() < 2 ||
      !provider.validate(waypoints.front(), waypoints.front()).success) {
    result.message = "invalid joint route or start state";
    return result;
  }
  if(waypoints.size()>2) {
    const auto blended=blendJointCorners(waypoints,std::min(.15,5*config.joint_space_step_rad));
    if(materializeJointRoute(blended,goal,provider,config,result,cancel)) {
      result.success=true;result.free_space_joint_route=true;
      result.used_detour=result.used_joint_space_detour=true;
      result.message="validated blended joint route";return result;
    }
    if(cancel && cancel())return result;
    result={};
  }
  result.success = materializeJointRoute(waypoints, goal, provider, config, result, cancel);
  result.free_space_joint_route = result.success;
  result.used_detour = waypoints.size() > 2;
  result.used_joint_space_detour = result.used_detour;
  result.message = result.success ? "validated joint route" : result.direct_failure_message;
  return result;
}

CartesianRoutePlan
CartesianRoutePlanner::plan(const Pose &start, const Pose &goal,
                            const KinematicState &seed, IKProvider &provider,
                            const CartesianRoutePlannerConfig &config,
                            const CancelCheck &cancel) {
  CartesianRoutePlan result;
  if (!validConfig(config)) {
    result.message = "Cartesian route planner configuration is invalid";
    return result;
  }

  if (config.mode == CartesianRouteMode::FREE_SPACE_JOINT) {
    if (jointSpaceDetour(start, goal, seed, provider, config, result, cancel)) {
      result.free_space_joint_route = true;
      if (!result.used_cylindrical_detour) {
        result.used_detour = false;
        result.message = "success: free-space joint interpolation";
      }
      return result;
    }
    result.message = cancel && cancel()
                         ? "Free-space joint route planning canceled"
                         : "No valid free-space joint route to target pose";
    if (!result.direct_failure_message.empty()) {
      result.message += ": " + result.direct_failure_message;
    }
    return result;
  }

  std::string path_error;
  if (!CartesianPath::linear(start, goal, config.path, result.cartesian_path,
                             &path_error)) {
    result.message = path_error;
    return result;
  }
  result.continuous_ik = ContinuousIK::solve(
      result.cartesian_path, seed, provider, config.continuous_ik, cancel);
  if (result.continuous_ik.success) {
    result.success = true;
    result.message = "success";
    return result;
  }
  result.direct_failure_index = result.continuous_ik.failure_index;
  result.direct_failure_message = result.continuous_ik.message;
  if (!config.singularity_avoidance_enabled ||
      config.detour_clearances_m.empty() || (cancel && cancel())) {
    result.message = result.direct_failure_message;
    return result;
  }

  std::vector<Pose> alternate_path;
  if (CartesianPath::alternateOrientation(start, goal, config.path,
                                          alternate_path, &path_error)) {
    auto alternate_ik = ContinuousIK::solve(alternate_path, seed, provider,
                                            config.continuous_ik, cancel);
    if (alternate_ik.success) {
      result.success = true;
      result.used_detour = true;
      result.used_alternate_orientation = true;
      result.cartesian_path = std::move(alternate_path);
      result.continuous_ik = std::move(alternate_ik);
      result.message = "success: alternate orientation arc";
      return result;
    }
  }

  if (jointSpaceDetour(start, goal, seed, provider, config, result, cancel)) {
    return result;
  }

  const double denominator = static_cast<double>(
      std::max<std::size_t>(1, result.cartesian_path.size() - 1));
  const double failed_fraction =
      std::clamp(static_cast<double>(result.direct_failure_index) / denominator,
                 0.10, 0.90);
  const auto directions = detourDirections(start, goal);
  for (const double clearance : config.detour_clearances_m) {
    for (const auto &direction : directions) {
      if (cancel && cancel()) {
        result.message = "Cartesian route planning canceled";
        return result;
      }
      ++result.detour_attempts;
      std::array<double, 3> offset{};
      for (std::size_t axis = 0; axis < offset.size(); ++axis) {
        offset[axis] = clearance * direction[axis];
      }
      for (const bool staged : {true, false}) {
        if (!staged) {
          ++result.detour_attempts;
        }
        std::vector<Pose> candidate_path;
        const bool path_created =
            staged ? CartesianPath::stagedDetour(start, goal, failed_fraction,
                                                 offset, config.path,
                                                 candidate_path, &path_error)
                   : CartesianPath::detour(start, goal, failed_fraction, offset,
                                           config.path, candidate_path,
                                           &path_error);
        if (!path_created) {
          continue;
        }
        auto candidate_ik = ContinuousIK::solve(candidate_path, seed, provider,
                                                config.continuous_ik, cancel);
        if (!candidate_ik.success) {
          const double failure_fraction =
              static_cast<double>(candidate_ik.failure_index) /
              static_cast<double>(
                  std::max<std::size_t>(1, candidate_path.size() - 1));
          if (failure_fraction > result.best_detour_failure_fraction) {
            result.best_detour_failure_fraction = failure_fraction;
            result.best_detour_failure_index = candidate_ik.failure_index;
            result.best_detour_failure_message = candidate_ik.message;
          }
          continue;
        }
        result.success = true;
        result.used_detour = true;
        result.detour_clearance_m = clearance;
        result.detour_direction = direction;
        result.cartesian_path = std::move(candidate_path);
        result.continuous_ik = std::move(candidate_ik);
        result.message = staged ? "success: staged kinematic detour"
                                : "success: smooth kinematic detour";
        return result;
      }
    }
  }

  result.message = "direct route failed (" + result.direct_failure_message +
                   "); no configured kinematic detour was feasible";
  if (!result.best_detour_failure_message.empty()) {
    result.message += "; best detour reached fraction=" +
                      std::to_string(result.best_detour_failure_fraction) +
                      " then failed (" + result.best_detour_failure_message +
                      ")";
  }
  return result;
}

} // namespace fr10_motion
