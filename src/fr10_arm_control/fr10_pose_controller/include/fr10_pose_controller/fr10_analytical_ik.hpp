#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "fr10_pose_controller/ik_provider.hpp"

namespace fr10_motion {

struct Fr10AnalyticalIKConfig {
  // Exact six-joint geometry represented by fairino10_v6.urdf.
  double shoulder_height_m{0.180};
  double upper_arm_length_m{0.700};
  double forearm_length_m{0.586};
  double wrist_1_offset_m{0.159};
  double wrist_2_offset_m{0.114};
  Pose wrist3_to_tip{{0.060, 0.0, 0.268}, {0.0, 0.0, 0.0, 1.0}};

  double min_target_reach_m{0.060};
  double max_target_reach_m{1.750};
  double wrist_min_abs_rad{0.0};
  double elbow_min_abs_rad{0.0};
  double max_wrist_reach_m{1.30};
  // Engineering exclusion cylinder. It must be disabled (0) or exceed the
  // 159 mm shoulder offset; a smaller positive value is geometrically
  // unreachable and therefore cannot enforce a radial barrier.
  double min_wrist_radius_m{0.200};
  double max_jacobian_condition{30.0};
  double geometric_position_tolerance_m{1e-7};
  double geometric_orientation_tolerance_rad{1e-7};
  std::array<double, kJointCount> seed_weights{{1.0, 1.0, 1.0, 0.5,
                                                0.5, 0.25}};
  KinematicLimits limits;
};

struct Fr10AnalyticalIKSolutions {
  std::array<KinematicState, 8> states{};
  std::size_t count{0};
};

struct Fr10AnalyticalIKStatistics {
  uint64_t solve_count{0};
  double mean_solve_time_us{0.0};
  double max_solve_time_us{0.0};
};

// Closed-form branch enumerator for the UR-style FR10 chain in the repository
// model. It has no ROS, MoveIt, network, or heap-backed branch container.
class Fr10AnalyticalIKProvider final : public IKProvider {
public:
  explicit Fr10AnalyticalIKProvider(Fr10AnalyticalIKConfig config);

  IKResult solve(const Pose &target, const KinematicState &seed) override;
  std::size_t solveCandidates(
      const Pose &target, const KinematicState &seed,
      std::array<IKResult, IKProvider::kMaxCandidates> &candidates,
      std::string *error = nullptr) override;
  IKResult validate(const KinematicState &target,
                    const KinematicState &seed) override;
  bool forward(const KinematicState &state, Pose &pose,
               std::string *error = nullptr) const override;

  bool solveAll(const Pose &target, const KinematicState &seed,
                Fr10AnalyticalIKSolutions &solutions,
                std::string *error = nullptr) const;
  bool workspaceReachable(const Pose &target) const;
  Fr10AnalyticalIKStatistics statistics() const;
  const Fr10AnalyticalIKConfig &config() const { return config_; }

private:
  class ScopedSolveTimer;

  bool validateState(const KinematicState &target,
                     const KinematicState &seed,
                     double &manipulability, double &condition_number,
                     std::string &error) const;
  void recordSolveTime(uint64_t nanoseconds);

  Fr10AnalyticalIKConfig config_;
  std::atomic<uint64_t> solve_count_{0};
  std::atomic<uint64_t> total_solve_ns_{0};
  std::atomic<uint64_t> max_solve_ns_{0};
};

} // namespace fr10_motion
