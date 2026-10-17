#include <iostream>
#include <limits>
#include "fr10_force_control/force_position_controller.hpp"
#include "fr10_motion_core/math_utils.hpp"

int main() {
  using namespace fr10_force;
  using namespace fr10_motion::math;
  bool ok = true;
  auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  AdmittanceConfig cfg;
  cfg.selected_axes.fill(true);
  cfg.stiffness = {2000., 2000., 0., 10., 10., 10.};
  cfg.damping = {1000., 1000., 100000., 20., 20., 20.};
  cfg.drive_deadband = {.05, .05, 0., 0., 0., 0.};
  cfg.max_correction = {.003, .003, .002, .05, .05, .05};
  AdmittanceModel model(cfg);
  Vector6 bias{.2, -.2, 0., 0., 0., 0.};
  Vector6 equilibrium{.0015, -.001, 0., .02, -.015, 0.};
  for (int i = 0; i < 12500; ++i) {
    const auto proposed = model.propose(bias, .008, equilibrium);
    check(proposed.valid && model.commit(proposed), "VALID_BIASED_ADMITTANCE");
  }
  check(std::abs(model.state().correction[0] - equilibrium[0]) < .000126 &&
        std::abs(model.state().correction[1] - equilibrium[1]) < .000126,
        "POSITIVE_XY_STIFFNESS_BOUNDS_100_SECONDS_BIAS_DRIFT");
  check(std::abs(model.state().correction[3] - equilibrium[3]) < 1e-9 &&
        std::abs(model.state().correction[4] - equilibrium[4]) < 1e-9,
        "SO3_EQUILIBRIUM_CONVERGENCE");
  const auto before = model.state();
  Vector6 different = equilibrium; different[0] = -.002;
  const auto uncommitted = model.propose(bias, .008, different);
  check(uncommitted.valid && model.state().correction == before.correction,
        "REFERENCE_PROPOSAL_HAS_NO_STATE_SIDE_EFFECT");
  const auto same = model.propose(bias, .008, equilibrium);
  check(std::abs(same.output.position[0] - before.correction[0]) < 1e-10,
        "DISCARDED_REFERENCE_CANNOT_REANCHOR_STATE");
  check(model.commit(same) && !model.canCommit(uncommitted) && !model.commit(uncommitted),
        "STALE_ADMITTANCE_PROPOSAL_CANNOT_OVERWRITE_ACCEPTED_STATE");
  // Noncommuting Rx/Ry rotations produce a real third-axis spring error.
  AdmittanceState rotated;
  rotated.correction = {0., 0., 0., .025, .01, .02};
  model.reset(rotated);
  const auto proposal = model.propose({}, .008, equilibrium);
  const auto rotation_error = quaternionToRotationVector(quaternionMultiply(
      rotationVectorToQuaternion({.025, .01, .02}),
      quaternionConjugate(rotationVectorToQuaternion({.02, -.015, 0.}))));
  const double denominator = cfg.mass[5] + .008 * cfg.damping[5] + .008*.008*cfg.stiffness[5];
  check(std::abs(proposal.output.velocity[5] -
        (-.008*cfg.stiffness[5]*rotation_error[2]/denominator)) < 1e-12 &&
        std::abs(rotation_error[2] - .02) > 1e-5,
        "ROTATIONAL_SPRING_USES_QUATERNION_ERROR_NOT_EULER_DIFFERENCE");
  CartesianPose nominal;
  nominal.orientation_xyzw = rotationVectorToQuaternion({.5, -.4, .3});
  const RotationMatrix identity{1.,0.,0.,0.,1.,0.,0.,0.,1.};
  ForcePositionController controller(cfg, identity);
  controller.reset(rotated);
  auto positive = controller.propose(nominal, {}, {}, .008, equilibrium);
  for (auto &v : nominal.orientation_xyzw) v = -v;
  auto negative = controller.propose(nominal, {}, {}, .008, equilibrium);
  check(positive.valid && negative.valid &&
        quaternionAngularDistance(positive.corrected_pose.orientation_xyzw,
                                  negative.corrected_pose.orientation_xyzw) < 1e-12,
        "QUATERNION_SIGN_INVARIANT_CORRECTION");
  equilibrium[0] = std::numeric_limits<double>::quiet_NaN();
  check(!model.propose({}, .008, equilibrium).valid, "INVALID_REFERENCE_REJECTED");
  return ok ? 0 : 1;
}
