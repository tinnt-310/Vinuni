#include <cmath>
#include <iostream>
#include "fr10_simulation_hardware/contact_wrench.hpp"

int main() {
  using fr10_simulation::ContactWrenchAccumulator;
  bool ok = true;
  const auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  ContactWrenchAccumulator unloaded;
  const auto empty = unloaded.inFrame(Eigen::Matrix3d::Identity());
  for (double value : empty) check(value == 0., "EXTERNAL_SOURCE_HAS_NO_GRAVITY_OR_SYNTHETIC_LOAD");
  ContactWrenchAccumulator sum;
  const Eigen::Vector3d tcp(1.,2.,3.), com(1.01,2.02,3.03), point(1.04,2.05,3.06);
  const Eigen::Vector3d applied_on_tool(0.,0.,-5.);
  const Eigen::Vector3d torsional_moment(.001,.002,.003);
  sum.add(applied_on_tool, (point-com).cross(applied_on_tool)+torsional_moment, com, tcp);
  const auto result = sum.inFrame(Eigen::Matrix3d::Identity());
  const Eigen::Vector3d expected = -(point-tcp).cross(applied_on_tool)-torsional_moment;
  check(result[2] == 5., "APPLIED_TOOL_CONTACT_SIGN_AT_TCP");
  for (std::size_t i=0; i<3; ++i)
    check(std::abs(result[i+3]-expected[i]) < 1e-14, "COM_TO_TCP_MOMENT_SHIFTED_EXACTLY_ONCE");
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto local = sum.inFrame(rotation);
  for (std::size_t i=0; i<3; ++i) {
    check(std::abs(local[i]-(rotation.transpose()*(-applied_on_tool))[i]) < 1e-14,
        "FORCE_AND_TORQUE_USE_THE_SAME_PHYSICAL_TCP_FRAME");
    check(std::abs(local[i+3]-(rotation.transpose()*expected)[i]) < 1e-14,
        "MOMENT_ROTATION_PRESERVES_POINT_OF_APPLICATION");
  }
  sum.add(-applied_on_tool, -(point-com).cross(applied_on_tool)-torsional_moment, com, tcp);
  for (double value : sum.inFrame(Eigen::Matrix3d::Identity()))
    check(std::abs(value) < 1e-14, "OPPOSITE_BODY_LOADS_CANCEL_INTERNAL_WRENCH");
  sum.add({std::numeric_limits<double>::quiet_NaN(),0.,0.}, Eigen::Vector3d::Zero(), com, tcp);
  for (double value : sum.inFrame(Eigen::Matrix3d::Identity()))
    check(std::isnan(value), "INVALID_CONTACT_CANNOT_BECOME_A_FRESH_ZERO");
  for (double value : unloaded.inFrame(Eigen::Matrix3d::Zero()))
    check(std::isnan(value), "INVALID_PHYSICAL_FRAME_REJECTED");
  return ok ? 0 : 1;
}
