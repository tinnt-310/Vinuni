#pragma once

#include "fr10_force_control/admittance_model.hpp"

namespace fr10_force {

struct ForcePositionResult {
  bool valid{false};
  CartesianPose corrected_pose;
  Vector6 force_error{};
  CartesianCorrection correction;
  AdmittanceProposal proposal;
};

class ForcePositionController {
public:
  ForcePositionController(AdmittanceConfig config,
                          RotationMatrix rotation_base_from_insertion);

  [[nodiscard]] ForcePositionResult propose(
      const CartesianPose &nominal_pose, const Wrench &desired_wrench,
      const Wrench &measured_wrench, double dt_s) const;
  [[nodiscard]] ForcePositionResult propose(
      const CartesianPose &nominal_pose, const Wrench &desired_wrench,
      const Wrench &measured_wrench, double dt_s,
      const Vector6 &equilibrium) const;
  bool commit(const ForcePositionResult &result);
  bool canCommit(const ForcePositionResult &result) const;
  void reset(const AdmittanceState &state = {});

  void setConfig(const AdmittanceConfig &config) { model_.setConfig(config); }

  const AdmittanceModel &model() const { return model_; }

private:
  AdmittanceModel model_;
  RotationMatrix rotation_base_from_insertion_;
};

} // namespace fr10_force
