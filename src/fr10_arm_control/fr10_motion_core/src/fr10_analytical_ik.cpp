#include "fr10_motion_core/fr10_analytical_ik.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kGeometryEpsilon = 1e-10;

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

Vector3 toEigen(const std::array<double, 3> &vector) {
  return Eigen::Map<const Vector3>(vector.data());
}

std::array<double, 3> toArray(const Vector3 &vector) {
  return {vector.x(), vector.y(), vector.z()};
}

bool quaternionToMatrix(std::array<double, 4> quaternion, Matrix3 &matrix) {
  if (!math::normalizeQuaternion(quaternion)) {
    return false;
  }
  const Eigen::Quaterniond rotation(quaternion[3], quaternion[0],
                                    quaternion[1], quaternion[2]);
  matrix = rotation.toRotationMatrix();
  return matrix.allFinite();
}

std::array<double, 4> matrixToQuaternion(const Matrix3 &matrix) {
  Eigen::Quaterniond quaternion(matrix);
  quaternion.normalize();
  std::array<double, 4> result{
      quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
  if (!math::normalizeQuaternion(result)) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return result;
}

double wrapToPi(double angle) {
  angle = std::remainder(angle, kTwoPi);
  return angle <= -kPi ? angle + kTwoPi : angle;
}

bool equivalentWithinLimits(double angle, double seed, double lower,
                            double upper, double &selected) {
  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  const double center = std::round((seed - angle) / kTwoPi);
  for (int offset = -2; offset <= 2; ++offset) {
    const double candidate = angle + (center + offset) * kTwoPi;
    if (candidate < lower - 1e-10 || candidate > upper + 1e-10) {
      continue;
    }
    const double distance = std::abs(candidate - seed);
    if (distance < best_distance) {
      best_distance = distance;
      selected = candidate;
      found = true;
    }
  }
  return found;
}

struct ForwardDetails {
  Pose tip_pose;
  Vector3 wrist3_position{Vector3::Zero()};
  std::array<Vector3, kJointCount> joint_origins;
  std::array<Vector3, kJointCount> joint_axes;
};

bool forwardDetails(const Fr10AnalyticalIKConfig &config,
                    const KinematicState &state, ForwardDetails &details) {
  if (!std::all_of(state.position_rad.begin(), state.position_rad.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }
  Matrix3 tool_rotation;
  if (!quaternionToMatrix(config.wrist3_to_tip.orientation_quat,
                          tool_rotation)) {
    return false;
  }
  Matrix3 rotation = Matrix3::Identity();
  Vector3 position = Vector3::Zero();
  const Vector3 local_z = Vector3::UnitZ();
  auto setJoint = [&](std::size_t joint) {
    details.joint_origins[joint] = position;
    details.joint_axes[joint] = rotation * local_z;
    rotation *= Eigen::AngleAxisd(state.position_rad[joint], local_z)
                    .toRotationMatrix();
  };
  auto translate = [&](const Vector3 &translation) {
    position += rotation * translation;
  };

  setJoint(0);
  translate(Vector3(0.0, 0.0, config.shoulder_height_m));
  rotation *= Eigen::AngleAxisd(kHalfPi, Vector3::UnitX()).toRotationMatrix();
  setJoint(1);
  translate(Vector3(-config.upper_arm_length_m, 0.0, 0.0));
  setJoint(2);
  translate(Vector3(-config.forearm_length_m, 0.0, 0.0));
  setJoint(3);
  translate(Vector3(0.0, 0.0, config.wrist_1_offset_m));
  rotation *= Eigen::AngleAxisd(kHalfPi, Vector3::UnitX()).toRotationMatrix();
  setJoint(4);
  translate(Vector3(0.0, 0.0, config.wrist_2_offset_m));
  rotation *= Eigen::AngleAxisd(-kHalfPi, Vector3::UnitX()).toRotationMatrix();
  setJoint(5);
  details.wrist3_position = position;
  translate(toEigen(config.wrist3_to_tip.position_m));
  rotation *= tool_rotation;
  details.tip_pose.position_m = toArray(position);
  details.tip_pose.orientation_quat = matrixToQuaternion(rotation);
  return position.allFinite();
}

} // namespace

class Fr10AnalyticalIKProvider::ScopedSolveTimer {
public:
  explicit ScopedSolveTimer(Fr10AnalyticalIKProvider &provider)
      : provider_(provider), start_(std::chrono::steady_clock::now()) {}

  ~ScopedSolveTimer() {
    provider_.recordSolveTime(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start_)
            .count()));
  }

private:
  Fr10AnalyticalIKProvider &provider_;
  std::chrono::steady_clock::time_point start_;
};

Fr10AnalyticalIKProvider::Fr10AnalyticalIKProvider(
    Fr10AnalyticalIKConfig config)
    : config_(std::move(config)) {
  const bool valid_geometry = config_.shoulder_height_m >= 0.0 &&
      config_.upper_arm_length_m > 0.0 &&
      config_.forearm_length_m > 0.0 && config_.wrist_1_offset_m > 0.0 &&
      config_.wrist_2_offset_m > 0.0 && config_.min_target_reach_m >= 0.0 &&
      config_.max_target_reach_m > config_.min_target_reach_m &&
      config_.max_wrist_reach_m > 0.0 &&
      (config_.min_wrist_radius_m == 0.0 ||
       config_.min_wrist_radius_m > config_.wrist_1_offset_m +
               kGeometryEpsilon) &&
      config_.max_jacobian_condition > 1.0 &&
      math::quaternionNormSquared(config_.wrist3_to_tip.orientation_quat) >
          1e-12;
  if (!valid_geometry) {
    throw std::invalid_argument("FR10 analytical IK configuration is invalid");
  }
}

bool Fr10AnalyticalIKProvider::workspaceReachable(const Pose &target) const {
  if (!std::all_of(target.position_m.begin(), target.position_m.end(),
                   [](double value) { return std::isfinite(value); }) ||
      math::quaternionNormSquared(target.orientation_quat) <= 1e-12) {
    return false;
  }
  const double reach = toEigen(target.position_m).norm();
  return reach >= config_.min_target_reach_m &&
         reach <= config_.max_target_reach_m;
}

bool Fr10AnalyticalIKProvider::solveAll(
    const Pose &target, const KinematicState &seed,
    Fr10AnalyticalIKSolutions &solutions, std::string *error) const {
  solutions = Fr10AnalyticalIKSolutions{};
  if (!workspaceReachable(target)) {
    if (error != nullptr) {
      *error = "target rejected by spherical workspace bounds";
    }
    return false;
  }
  Matrix3 target_rotation;
  Matrix3 tool_rotation;
  if (!quaternionToMatrix(target.orientation_quat, target_rotation) ||
      !quaternionToMatrix(config_.wrist3_to_tip.orientation_quat,
                          tool_rotation)) {
    if (error != nullptr) {
      *error = "target or tool quaternion is invalid";
    }
    return false;
  }
  const Matrix3 wrist3_rotation = target_rotation * tool_rotation.transpose();
  const Vector3 wrist3_position =
      toEigen(target.position_m) -
      wrist3_rotation * toEigen(config_.wrist3_to_tip.position_m);
  const double x = wrist3_position[0];
  const double y = wrist3_position[1];
  const double z = wrist3_position[2];
  const double radius = std::hypot(x, y);
  if (radius + kGeometryEpsilon < config_.wrist_1_offset_m) {
    if (error != nullptr) {
      *error = "wrist center is inside the shoulder-offset cylinder";
    }
    return false;
  }

  const double azimuth = std::atan2(y, x);
  const double shoulder_offset = std::asin(std::clamp(
      config_.wrist_1_offset_m / std::max(radius, kGeometryEpsilon),
      -1.0, 1.0));
  const std::array<double, 2> shoulder_candidates{
      azimuth + shoulder_offset, azimuth + kPi - shoulder_offset};
  const Vector3 tool_z = wrist3_rotation.col(2);

  for (const double shoulder_raw : shoulder_candidates) {
    const double shoulder = wrapToPi(shoulder_raw);
    const double c1 = std::cos(shoulder);
    const double s1 = std::sin(shoulder);
    double c5 = s1 * tool_z[0] - c1 * tool_z[1];
    if (c5 < -1.0 - 1e-9 || c5 > 1.0 + 1e-9) {
      continue;
    }
    c5 = std::clamp(c5, -1.0, 1.0);
    const double wrist_angle = std::acos(c5);
    const std::array<double, 2> wrist_candidates{{wrist_angle,
                                                   -wrist_angle}};
    for (const double q5_raw : wrist_candidates) {
      const double s5 = std::sin(q5_raw);
      if (std::abs(s5) <= kGeometryEpsilon) {
        continue;
      }
      const double radial_tool_z = c1 * tool_z[0] + s1 * tool_z[1];
      const double q234 =
          std::atan2(-tool_z[2] / s5, -radial_tool_z / s5);
      const double radial = c1 * x + s1 * y;
      const double planar_x =
          config_.wrist_2_offset_m * std::sin(q234) - radial;
      const double planar_z = config_.shoulder_height_m - z -
                              config_.wrist_2_offset_m * std::cos(q234);
      double c3 =
          (planar_x * planar_x + planar_z * planar_z -
           config_.upper_arm_length_m * config_.upper_arm_length_m -
           config_.forearm_length_m * config_.forearm_length_m) /
          (2.0 * config_.upper_arm_length_m *
           config_.forearm_length_m);
      if (c3 < -1.0 - 1e-9 || c3 > 1.0 + 1e-9) {
        continue;
      }
      c3 = std::clamp(c3, -1.0, 1.0);
      const double elbow_angle = std::acos(c3);
      const std::array<double, 2> elbow_candidates{{elbow_angle,
                                                    -elbow_angle}};
      for (const double q3_raw : elbow_candidates) {
        const double q2_raw =
            std::atan2(planar_z, planar_x) -
            std::atan2(config_.forearm_length_m * std::sin(q3_raw),
                       config_.upper_arm_length_m +
                           config_.forearm_length_m * std::cos(q3_raw));
        const double q4_raw = q234 - q2_raw - q3_raw;
        const Matrix3 before_q6 =
            Eigen::AngleAxisd(shoulder, Vector3::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(kHalfPi, Vector3::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(q234, Vector3::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(kHalfPi, Vector3::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(q5_raw, Vector3::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(-kHalfPi, Vector3::UnitX()).toRotationMatrix();
        const Matrix3 q6_rotation =
            before_q6.transpose() * wrist3_rotation;
        const double q6_raw =
            std::atan2(q6_rotation(1, 0), q6_rotation(0, 0));
        const std::array<double, kJointCount> raw{
            {shoulder, q2_raw, q3_raw, q4_raw, q5_raw, q6_raw}};
        KinematicState candidate;
        bool within_limits = true;
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
          if (!equivalentWithinLimits(
                  raw[joint], seed.position_rad[joint],
                  config_.limits.min_joint_position_rad[joint],
                  config_.limits.max_joint_position_rad[joint],
                  candidate.position_rad[joint])) {
            within_limits = false;
            break;
          }
        }
        if (!within_limits) {
          continue;
        }
        bool duplicate = false;
        for (std::size_t index = 0; index < solutions.count; ++index) {
          double maximum_difference = 0.0;
          for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            maximum_difference = std::max(
                maximum_difference,
                std::abs(solutions.states[index].position_rad[joint] -
                         candidate.position_rad[joint]));
          }
          duplicate = duplicate || maximum_difference < 1e-8;
        }
        if (!duplicate && solutions.count < solutions.states.size()) {
          solutions.states[solutions.count++] = candidate;
        }
      }
    }
  }
  if (solutions.count == 0 && error != nullptr) {
    *error = "closed-form IK found no branch within configured limits";
  }
  return solutions.count > 0;
}

bool Fr10AnalyticalIKProvider::validateState(
    const KinematicState &target, const KinematicState &seed,
    double &manipulability, double &condition_number,
    std::string &error) const {
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (!std::isfinite(target.position_rad[joint]) ||
        target.position_rad[joint] <
            config_.limits.min_joint_position_rad[joint] ||
        target.position_rad[joint] >
            config_.limits.max_joint_position_rad[joint]) {
      error = "joint limit violation";
      return false;
    }
  }
  if (!barrierSafe(seed.position_rad[4], target.position_rad[4],
                   config_.wrist_min_abs_rad)) {
    error = "wrist singularity barrier violated";
    return false;
  }
  if (!barrierSafe(seed.position_rad[2], target.position_rad[2],
                   config_.elbow_min_abs_rad)) {
    error = "elbow singularity barrier violated";
    return false;
  }

  ForwardDetails details;
  if (!forwardDetails(config_, target, details) ||
      details.wrist3_position.norm() > config_.max_wrist_reach_m ||
      std::hypot(details.wrist3_position[0], details.wrist3_position[1]) <
          config_.min_wrist_radius_m) {
    error = "workspace/singularity wrist-link barrier violated";
    return false;
  }
  Eigen::Matrix<double, 6, 6> jacobian;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    const Vector3 linear = details.joint_axes[joint].cross(
        toEigen(details.tip_pose.position_m) - details.joint_origins[joint]);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      jacobian(static_cast<Eigen::Index>(axis),
               static_cast<Eigen::Index>(joint)) = linear[axis];
      jacobian(static_cast<Eigen::Index>(axis + 3),
               static_cast<Eigen::Index>(joint)) =
          details.joint_axes[joint][axis];
    }
  }
  if (!jacobian.allFinite()) {
    error = "invalid analytical Jacobian";
    return false;
  }
  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>,
                         Eigen::NoQRPreconditioner>
      svd(jacobian);
  const auto singular_values = svd.singularValues();
  const double minimum = singular_values(5);
  if (!(minimum > std::numeric_limits<double>::epsilon())) {
    error = "singular analytical Jacobian";
    return false;
  }
  condition_number = singular_values(0) / minimum;
  manipulability = std::abs(jacobian.determinant());
  if (!std::isfinite(condition_number) ||
      condition_number > config_.max_jacobian_condition) {
    error = "Jacobian condition-number limit exceeded";
    return false;
  }
  return true;
}

IKResult Fr10AnalyticalIKProvider::solve(const Pose &target,
                                         const KinematicState &seed) {
  ScopedSolveTimer timer(*this);
  IKResult result;
  Fr10AnalyticalIKSolutions solutions;
  std::string enumeration_error;
  if (!solveAll(target, seed, solutions, &enumeration_error)) {
    result.message = enumeration_error;
    return result;
  }

  std::array<std::pair<double, std::size_t>, 8> ranked{};
  for (std::size_t index = 0; index < solutions.count; ++index) {
    double cost = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double delta = solutions.states[index].position_rad[joint] -
                           seed.position_rad[joint];
      cost += config_.seed_weights[joint] * delta * delta;
    }
    ranked[index] = {cost, index};
  }
  std::sort(ranked.begin(), ranked.begin() +
                                static_cast<std::ptrdiff_t>(solutions.count));
  for (std::size_t rank = 0; rank < solutions.count; ++rank) {
    const std::size_t index = ranked[rank].second;
    double manipulability = 0.0;
    double condition_number = std::numeric_limits<double>::infinity();
    std::string validation_error;
    if (validateState(solutions.states[index], seed, manipulability,
                      condition_number, validation_error)) {
      result.success = true;
      result.state = solutions.states[index];
      result.manipulability = manipulability;
      result.condition_number = condition_number;
      break;
    }
  }
  result.message = result.success ? "success" :
                    "all analytical IK branches violate safety barriers";
  return result;
}

std::size_t Fr10AnalyticalIKProvider::solveCandidates(
    const Pose &target, const KinematicState &seed,
    std::array<IKResult, IKProvider::kMaxCandidates> &candidates,
    std::string *error) {
  ScopedSolveTimer timer(*this);
  candidates = {};
  Fr10AnalyticalIKSolutions solutions;
  std::string enumeration_error;
  if (!solveAll(target, seed, solutions, &enumeration_error)) {
    if (error != nullptr) {
      *error = enumeration_error;
    }
    return 0;
  }

  std::size_t accepted = 0;
  std::string last_validation_error;
  for (std::size_t index = 0;
       index < solutions.count && accepted < candidates.size(); ++index) {
    IKResult candidate;
    candidate.state = solutions.states[index];
    if (!validateState(candidate.state, seed, candidate.manipulability,
                       candidate.condition_number,
                       last_validation_error)) {
      continue;
    }
    candidate.success = true;
    candidate.message = "success";
    candidates[accepted++] = candidate;
  }
  if (accepted == 0 && error != nullptr) {
    *error = last_validation_error.empty()
                 ? "all analytical IK branches violate safety barriers"
                 : last_validation_error;
  }
  return accepted;
}

IKResult Fr10AnalyticalIKProvider::validate(
    const KinematicState &target, const KinematicState &seed) {
  IKResult result;
  result.state = target;
  if (!validateState(target, seed, result.manipulability,
                     result.condition_number, result.message)) {
    return result;
  }
  result.success = true;
  result.message = "success";
  return result;
}

bool Fr10AnalyticalIKProvider::forward(const KinematicState &state, Pose &pose,
                                       std::string *error) const {
  ForwardDetails details;
  if (!forwardDetails(config_, state, details)) {
    if (error != nullptr) {
      *error = "analytical FK input or result is invalid";
    }
    return false;
  }
  pose = details.tip_pose;
  return true;
}

void Fr10AnalyticalIKProvider::recordSolveTime(uint64_t nanoseconds) {
  solve_count_.fetch_add(1, std::memory_order_relaxed);
  total_solve_ns_.fetch_add(nanoseconds, std::memory_order_relaxed);
  uint64_t previous = max_solve_ns_.load(std::memory_order_relaxed);
  while (previous < nanoseconds &&
         !max_solve_ns_.compare_exchange_weak(
             previous, nanoseconds, std::memory_order_relaxed)) {
  }
}

Fr10AnalyticalIKStatistics Fr10AnalyticalIKProvider::statistics() const {
  Fr10AnalyticalIKStatistics result;
  result.solve_count = solve_count_.load(std::memory_order_relaxed);
  const uint64_t total = total_solve_ns_.load(std::memory_order_relaxed);
  result.mean_solve_time_us =
      result.solve_count > 0
          ? static_cast<double>(total) /
                static_cast<double>(result.solve_count) / 1000.0
          : 0.0;
  result.max_solve_time_us =
      static_cast<double>(max_solve_ns_.load(std::memory_order_relaxed)) /
      1000.0;
  return result;
}

} // namespace fr10_motion
