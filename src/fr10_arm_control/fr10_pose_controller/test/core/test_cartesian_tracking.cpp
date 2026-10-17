#include "fr10_motion_core/cartesian_tracking.hpp"
#include "test_support.hpp"

#include <limits>

int main() {
  fr10_motion_test::Check check;
  fr10_motion::Pose reference;
  reference.orientation_quat = {0.0, 0.0, 0.0, 1.0};
  fr10_motion::Pose actual = reference;
  actual.position_m = {0.03, 0.04, 0.0};
  const auto errors = fr10_motion::cartesianTrackingErrors(
      reference, {1.0, 0.0, 0.0}, actual);
  check.require(fr10_motion_test::near(errors.position_error_m, 0.05, 1e-12),
                "position", "Euclidean displacement");
  check.require(fr10_motion_test::near(errors.along_track_error_m, 0.03, 1e-12),
                "along", "along track displacement");
  check.require(fr10_motion_test::near(errors.perpendicular_error_m, 0.04, 1e-12),
                "perpendicular", "cross track displacement");
  check.require(fr10_motion_test::near(errors.orientation_error_rad, 0.0, 1e-12),
                "orientation", "identical orientations");
  actual.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = fr10_motion::cartesianTrackingErrors(
      reference, {1.0, 0.0, 0.0}, actual);
  check.require(std::isinf(invalid.position_error_m) &&
                    std::isinf(invalid.perpendicular_error_m),
                "nonfinite", "invalid feedback must fail closed");
  return check.result();
}
