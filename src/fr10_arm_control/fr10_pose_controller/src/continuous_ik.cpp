#include "fr10_pose_controller/continuous_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion {

ContinuousIKResult ContinuousIK::solve(
    const std::vector<Pose> &cartesian_path, const KinematicState &seed,
    IKProvider &provider, const ContinuousIKConfig &config,
    const CancelCheck &cancel) {
  ContinuousIKResult result;
  result.min_manipulability = std::numeric_limits<double>::infinity();
  if (cartesian_path.empty() || !(config.max_joint_step_rad > 0.0) ||
      !(config.max_position_error_m >= 0.0) ||
      !(config.max_orientation_error_rad >= 0.0)) {
    result.message = "continuous IK input/configuration is invalid";
    return result;
  }

  struct SearchNode {
    KinematicState state;
    double cost{0.0};
    double max_position_error_m{0.0};
    double max_orientation_error_rad{0.0};
    double min_manipulability{std::numeric_limits<double>::infinity()};
    double max_condition_number{0.0};
    std::size_t parent{0};
  };
  constexpr std::size_t kBeamWidth = 16;
  std::vector<std::vector<SearchNode>> layers;
  layers.reserve(cartesian_path.size());
  SearchNode initial;
  initial.state = seed;
  layers.push_back({initial});

  for (std::size_t index = 1; index < cartesian_path.size(); ++index) {
    if (cancel && cancel()) {
      result.failure_index = index;
      result.message = "continuous IK canceled";
      return result;
    }
    std::vector<SearchNode> next;
    std::string last_error = "IK failed";
    bool continuity_rejected = false;
    double minimum_rejected_step = std::numeric_limits<double>::infinity();
    std::size_t minimum_rejected_joint = 0;
    for (std::size_t parent_index = 0;
         parent_index < layers.back().size(); ++parent_index) {
      const auto &parent = layers.back()[parent_index];
      std::array<IKResult, IKProvider::kMaxCandidates> candidates{};
      std::string solve_error;
      const std::size_t candidate_count = provider.solveCandidates(
          cartesian_path[index], parent.state, candidates, &solve_error);
      if (candidate_count == 0 && !solve_error.empty()) {
        last_error = solve_error;
      }
      for (std::size_t candidate_index = 0;
           candidate_index < candidate_count; ++candidate_index) {
        const IKResult &ik = candidates[candidate_index];
        bool continuous = ik.success;
        double transition_cost = 0.0;
        double maximum_step = 0.0;
        std::size_t maximum_step_joint = 0;
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
          const double delta = ik.state.position_rad[joint] -
                               parent.state.position_rad[joint];
          continuous = continuous &&
                       std::isfinite(ik.state.position_rad[joint]) &&
                       std::abs(delta) <= config.max_joint_step_rad;
          transition_cost += delta * delta;
          if (std::abs(delta) > maximum_step) {
            maximum_step = std::abs(delta);
            maximum_step_joint = joint;
          }
        }
        if (!continuous) {
          continuity_rejected = true;
          if (maximum_step < minimum_rejected_step) {
            minimum_rejected_step = maximum_step;
            minimum_rejected_joint = maximum_step_joint;
          }
          continue;
        }

        Pose reconstructed;
        std::string fk_error;
        if (!provider.forward(ik.state, reconstructed, &fk_error)) {
          last_error = fk_error.empty() ? "FK validation failed" : fk_error;
          continue;
        }
        double position_error_squared = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double error = reconstructed.position_m[axis] -
                               cartesian_path[index].position_m[axis];
          position_error_squared += error * error;
        }
        const double position_error = std::sqrt(position_error_squared);
        const double orientation_error = math::quaternionAngularDistance(
            reconstructed.orientation_quat,
            cartesian_path[index].orientation_quat);
        if (!std::isfinite(position_error) ||
            !std::isfinite(orientation_error) ||
            position_error > config.max_position_error_m ||
            orientation_error > config.max_orientation_error_rad) {
          last_error =
              "IK/FK Cartesian error exceeds configured tolerance";
          continue;
        }

        SearchNode node;
        node.state = ik.state;
        node.cost = parent.cost + transition_cost;
        node.max_position_error_m =
            std::max(parent.max_position_error_m, position_error);
        node.max_orientation_error_rad =
            std::max(parent.max_orientation_error_rad, orientation_error);
        node.min_manipulability =
            std::min(parent.min_manipulability, ik.manipulability);
        node.max_condition_number =
            std::max(parent.max_condition_number, ik.condition_number);
        node.parent = parent_index;

        bool replaced = false;
        for (auto &existing : next) {
          double maximum_difference = 0.0;
          for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            maximum_difference = std::max(
                maximum_difference,
                std::abs(existing.state.position_rad[joint] -
                         node.state.position_rad[joint]));
          }
          if (maximum_difference < 1e-8) {
            if (node.cost < existing.cost) {
              existing = node;
            }
            replaced = true;
            break;
          }
        }
        if (!replaced) {
          next.push_back(node);
        }
      }
    }
    if (next.empty()) {
      result.failure_index = index;
      result.message =
          continuity_rejected
              ? "IK continuity limit exceeded: minimum rejected step=" +
                    std::to_string(minimum_rejected_step) +
                    " rad at joint " +
                    std::to_string(minimum_rejected_joint + 1)
              : last_error;
      return result;
    }
    std::sort(next.begin(), next.end(),
              [](const SearchNode &left, const SearchNode &right) {
                return left.cost < right.cost;
              });
    if (next.size() > kBeamWidth) {
      next.resize(kBeamWidth);
    }
    layers.push_back(std::move(next));
  }

  const auto best = std::min_element(
      layers.back().begin(), layers.back().end(),
      [](const SearchNode &left, const SearchNode &right) {
        return left.cost < right.cost;
      });
  std::size_t selected =
      static_cast<std::size_t>(std::distance(layers.back().begin(), best));
  result.joint_path.resize(cartesian_path.size());
  for (std::size_t layer = layers.size(); layer-- > 0;) {
    result.joint_path[layer] = layers[layer][selected].state;
    if (layer > 0) {
      selected = layers[layer][selected].parent;
    }
  }
  result.max_position_error_m = best->max_position_error_m;
  result.max_orientation_error_rad = best->max_orientation_error_rad;
  result.min_manipulability = best->min_manipulability;
  result.max_condition_number = best->max_condition_number;
  result.success = true;
  result.message = "success";
  if (!std::isfinite(result.min_manipulability)) {
    result.min_manipulability = 0.0;
  }
  return result;
}

} // namespace fr10_motion
