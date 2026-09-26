#pragma once

#include <array>
#include <memory>
#include <ruckig/ruckig.hpp>

#include "fr10_pose_controller/core_types.hpp"

namespace fr10_motion {

class JointOtg {
public:
  JointOtg(double cycle_period_s, const KinematicLimits &limits,
           double max_reference_lead_rad = 0.017453292519943295769);
  ~JointOtg() = default;

  // Reset OTG from a static state (zero velocity/acceleration)
  void reset(const std::array<double, kJointCount> &initial_position);
  void reset(const std::array<double, kJointCount> &initial_position,
             const std::array<double, kJointCount> &initial_velocity,
             const std::array<double, kJointCount> &initial_acceleration);

  // Submit target: if currently moving, seamlessly retargets from generated
  // trajectory state
  bool submitTarget(const JointTarget &target);

  // Generate a bounded jerk-limited transition from the current generated
  // state to zero joint velocity. Position remains an output of the velocity-
  // controlled Ruckig profile; no instantaneous hold step is introduced.
  bool startControlledStop();

  // Advance OTG by one cycle (e.g., 8ms at 125Hz)
  // Returns true if trajectory calculation succeeded
  bool step(const RobotState &feedback,
            std::array<double, kJointCount> &out_position,
            std::array<double, kJointCount> &out_velocity,
            std::array<double, kJointCount> &out_acceleration);

  // Check if reached target
  [[nodiscard]] bool isFinished() const;

  // A lag hold is deliberately not reported as a Ruckig calculation failure:
  // the last bounded command is retained while the measured robot catches up.
  // These accessors make that supervisory state explicit instead of silently
  // presenting the held cycle as normal progress.
  [[nodiscard]] bool referenceLagging() const { return reference_lagging_; }
  [[nodiscard]] uint32_t referenceLagCycles() const {
    return reference_lag_cycles_;
  }
  [[nodiscard]] double referenceActualErrorRad() const {
    return reference_actual_error_rad_;
  }

  // Current active target revision
  uint64_t activeRevision() const { return active_revision_; }

  // Current generated state
  const std::array<double, kJointCount> &currentPosition() const {
    return current_position_;
  }
  const std::array<double, kJointCount> &currentVelocity() const {
    return current_velocity_;
  }
  const std::array<double, kJointCount> &currentAcceleration() const {
    return current_acceleration_;
  }

  // Dynamic limits update
  void setLimits(const KinematicLimits &limits);
  void setCyclePeriod(double period_s);
  void setMaxReferenceLead(double max_reference_lead_rad);

private:
  double cycle_period_s_{0.008};
  KinematicLimits limits_;
  double max_reference_lead_rad_{0.017453292519943295769};

  std::unique_ptr<ruckig::Ruckig<kJointCount>> ruckig_;
  ruckig::InputParameter<kJointCount> input_;
  ruckig::OutputParameter<kJointCount> output_;

  std::array<double, kJointCount> current_position_{};
  std::array<double, kJointCount> current_velocity_{};
  std::array<double, kJointCount> current_acceleration_{};

  uint64_t active_revision_{0};
  bool initialized_{false};
  bool finished_{true};
  bool reference_lagging_{false};
  uint32_t reference_lag_cycles_{0};
  double reference_actual_error_rad_{0.0};
  ruckig::Result last_result_{ruckig::Result::Finished};
};

} // namespace fr10_motion
