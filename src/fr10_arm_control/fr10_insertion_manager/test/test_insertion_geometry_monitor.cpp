#include <cmath>
#include <iostream>
#include <limits>

#include "fr10_insertion_manager/insertion_geometry_monitor.hpp"
#include "fr10_force_control/wrench_transform.hpp"
#include "fr10_motion_core/math_utils.hpp"

int main() {
  bool ok = true;
  const auto check = [&ok](bool pass, const char *name) {
    if (!pass) { ok = false; std::cerr << "FAIL " << name << '\n'; }
  };
  fr10_force::CartesianPose contact, actual;
  actual.position_m = {0.001, 0.0, 0.005};
  auto observation = fr10_insertion::observeInsertionGeometry(
      contact, {0,0,1}, actual, 0.002, 0.0174532925199433);
  check(observation.valid && observation.lateral_valid && observation.orientation_valid,
        "ALIGNED_MEASURED_FEEDBACK");
  check(std::abs(observation.depth_m - 0.005) < 1e-12, "ACTUAL_DEPTH");
  actual.position_m[0] = 0.0021;
  observation = fr10_insertion::observeInsertionGeometry(contact, {0,0,1}, actual, 0.002, 0.0174532925199433);
  check(!observation.lateral_valid, "LATERAL_LIMIT");
  actual.position_m = {0,0,-0.003};
  actual.orientation_xyzw = {0,0,std::sqrt(0.5),std::sqrt(0.5)};
  observation = fr10_insertion::observeInsertionGeometry(contact, {0,0,1}, actual, 0.002, 0.0174532925199433);
  check(observation.valid && !observation.orientation_valid && observation.depth_m < 0,
        "ORIENTATION_LIMIT_AND_SIGNED_PREINSERT");
  const auto wrench = fr10_force::WrenchTransform::apply({{1,0,0},{0,1,0}}, observation.insertion_from_tcp);
  check(std::abs(wrench.force_n[1]-1) < 1e-12 && std::abs(wrench.torque_nm[0]+1) < 1e-12,
        "MOVING_TCP_TO_FIXED_INSERTION_AXES");
  contact.orientation_xyzw = actual.orientation_xyzw;
  observation = fr10_insertion::observeInsertionGeometry(contact, {0,0,1}, actual, 0.002, 0.0174532925199433);
  check(observation.orientation_valid && std::abs(observation.insertion_from_tcp.rotation_target_from_source[0]-1) < 1e-12,
        "FIXED_INSERTION_FRAME_NOT_BASE_FRAME");
  actual.orientation_xyzw = {};
  check(!fr10_insertion::observeInsertionGeometry(contact, {0,0,1}, actual, 0.002, 0.02).valid,
        "ZERO_QUATERNION_FAIL_CLOSED");
  actual.orientation_xyzw = {0,0,0,1};
  actual.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  check(!fr10_insertion::observeInsertionGeometry(contact, {0,0,1}, actual, 0.002, 0.02).valid,
        "NONFINITE_MEASUREMENT_FAIL_CLOSED");
  actual.position_m = {};
  check(!fr10_insertion::observeInsertionGeometry(contact, {0,0,2}, actual, 0.002, 0.02).valid,
        "INVALID_AXIS_FAIL_CLOSED");
  fr10_insertion::InsertionGeometryObservation mating;
  mating.valid = true;
  mating.lateral_error_m = .0001; mating.orientation_error_rad = .002;
  check(fr10_insertion::withinMatingClearance(mating, .00025, .05),
        "TRANSLATION_AND_ROTATION_SHARE_MATING_CLEARANCE");
  mating.lateral_error_m = .0002;
  check(!fr10_insertion::withinMatingClearance(mating, .00025, .05),
        "INDEPENDENTLY_SMALL_ERRORS_CAN_EXCEED_COMBINED_CAD_CLEARANCE");
  mating.lateral_error_m = .001;
  check(!fr10_insertion::withinMatingClearance(mating, .00025, .05),
        "CAPTURE_TOLERANCE_IS_NOT_SEATED_TOLERANCE");
  check(!fr10_insertion::withinMatingClearance(mating, 0., .05), "MISSING_MATING_DATUM_FAILS_CLOSED");
  contact = {}; actual = {};
  actual.orientation_xyzw = fr10_motion::math::rotationVectorToQuaternion({0., 0., 1e-9});
  observation = fr10_insertion::observeInsertionGeometry(contact, {0.,0.,1.}, actual, .002, .02);
  check(observation.valid && std::abs(observation.orientation_error_rad - 1e-9) < 1e-15,
        "TINY_ROTATION_NOT_QUANTIZED_BY_ACOS_TRACE");
  auto quaternion = fr10_motion::math::rotationVectorToQuaternion({.2, -.4, .3});
  fr10_force::RotationMatrix matrix;
  check(fr10_motion::math::quaternionToRotationMatrix(quaternion, matrix), "SHARED_ROTATION_CONVERSION");
  for (std::size_t column = 0; column < 3; ++column) {
    fr10_force::Vector3 basis{}; basis[column] = 1.;
    const auto rotated = fr10_motion::math::rotateVector(quaternion, basis);
    for (std::size_t row = 0; row < 3; ++row)
      check(std::abs(rotated[row] - matrix[3*row+column]) < 1e-12, "MATRIX_MATCHES_QUATERNION_COMPOSITION");
  }
  const auto original = matrix;
  check(!fr10_motion::math::quaternionToRotationMatrix({}, matrix) && matrix == original,
        "INVALID_ROTATION_NOT_REPLACED_BY_IDENTITY");
  // Opposite input translation/tilt exaggerates distance to the original
  // ray while a measured-axis withdrawal stays in its accepted corridor.
  fr10_force::CartesianPose uncertain_input;
  uncertain_input.position_m={.002,0.,0.};
  uncertain_input.orientation_xyzw=fr10_motion::math::rotationVectorToQuaternion({0.,-.03490658503988659,0.});
  const auto input_axis=fr10_motion::math::rotateVector(uncertain_input.orientation_xyzw,{0.,0.,1.});
  fr10_force::CartesianPose seated;
  seated.position_m={-.00005,0.,.015};
  const auto measured=fr10_insertion::observeInsertionGeometry(uncertain_input,input_axis,seated,.003,.04);
  fr10_insertion::InsertionCorridor withdrawal{seated,{0.,0.,1.}};
  withdrawal.mouth_pose.position_m[2]-=measured.depth_m;
  auto withdrawn=withdrawal.mouth_pose;withdrawn.position_m[2]-=.030;
  const auto original_ray=fr10_insertion::observeInsertionGeometry(uncertain_input,input_axis,withdrawn,.003,.04);
  const auto accepted_ray=fr10_insertion::observeInsertionGeometry(withdrawal,withdrawn,.003,.04);
  check(original_ray.valid && !original_ray.lateral_valid && accepted_ray.valid &&
      accepted_ray.lateral_valid && accepted_ray.orientation_valid &&
      std::abs(accepted_ray.depth_m+.030)<1e-12,
      "WITHDRAWAL_DEPTH_AND_TRACKING_SHARE_MEASURED_AXIS_NOT_UNCERTAIN_INPUT");
  withdrawn.position_m[0]+=.0031;
  check(!fr10_insertion::observeInsertionGeometry(withdrawal,withdrawn,.003,.04).lateral_valid,
      "MEASURED_WITHDRAWAL_CORRIDOR_PRESERVES_TRACKING_LIMIT");
  return ok ? 0 : 1;
}
