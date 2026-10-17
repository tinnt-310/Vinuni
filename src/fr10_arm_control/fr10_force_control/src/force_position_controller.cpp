#include "fr10_force_control/force_position_controller.hpp"
#include "fr10_force_control/wrench_transform.hpp"
#include "fr10_motion_core/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_force {
namespace {

bool rotationFinite(const RotationMatrix &rotation) {
  RigidTransform frame;
  frame.rotation_target_from_source = rotation;
  return WrenchTransform::valid(frame);
}

} // namespace

ForcePositionController::ForcePositionController(
    AdmittanceConfig config,
    RotationMatrix rotation_base_from_insertion)
    : model_(config),
      rotation_base_from_insertion_(rotation_base_from_insertion) {
  if (!rotationFinite(rotation_base_from_insertion_)) {
    throw std::invalid_argument("insertion-frame rotation is invalid");
  }
}

ForcePositionResult ForcePositionController::propose(
    const CartesianPose &nominal_pose, const Wrench &desired_wrench,
    const Wrench &measured_wrench, double dt_s) const {
  return propose(nominal_pose, desired_wrench, measured_wrench, dt_s, {});
}

ForcePositionResult ForcePositionController::propose(
    const CartesianPose &nominal_pose, const Wrench &desired_wrench,
    const Wrench &measured_wrench, double dt_s, const Vector6 &equilibrium) const {
  using namespace fr10_motion::math;
  ForcePositionResult result;
  if (!finite(nominal_pose) || !finite(desired_wrench) ||
      !finite(measured_wrench)) {
    return result;
  }
  Quaternion nominal_orientation = nominal_pose.orientation_xyzw;
  if (!normalizeQuaternion(nominal_orientation)) {
    return result;
  }
  const auto desired = desired_wrench.vector();
  const auto measured = measured_wrench.vector();
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    result.force_error[axis] = desired[axis] - measured[axis];
  }
  result.proposal = model_.propose(result.force_error, dt_s, equilibrium);
  if (!result.proposal.valid) {
    return result;
  }
  result.correction = result.proposal.output;
  result.corrected_pose = nominal_pose;

  const Vector3 translation_in_insertion{
      result.correction.position[0], result.correction.position[1],
      result.correction.position[2]};
  const Vector3 translation_in_base = rotate(
      rotation_base_from_insertion_, translation_in_insertion);
  for (std::size_t axis = 0; axis < kTranslationDofs; ++axis) {
    result.corrected_pose.position_m[axis] += translation_in_base[axis];
  }

  const Vector3 rotation_in_insertion{
      result.correction.position[3], result.correction.position[4],
      result.correction.position[5]};
  const Vector3 rotation_in_base = rotate(
      rotation_base_from_insertion_, rotation_in_insertion);
  result.corrected_pose.orientation_xyzw = quaternionMultiply(
      rotationVectorToQuaternion(rotation_in_base), nominal_orientation);
  if (!normalizeQuaternion(result.corrected_pose.orientation_xyzw)) {
    return ForcePositionResult{};
  }
  result.valid = true;
  return result;
}

bool ForcePositionController::commit(const ForcePositionResult &result) {
  return result.valid && model_.commit(result.proposal);
}

bool ForcePositionController::canCommit(const ForcePositionResult &result) const {
  return result.valid && model_.canCommit(result.proposal);
}

void ForcePositionController::reset(const AdmittanceState &state) {
  model_.reset(state);
}

} // namespace fr10_force
