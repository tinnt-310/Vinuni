#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "swerve_vehicle_control/swerve_kinematics.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kOffset = 0.005062;
using swerve_vehicle_control::PlanarTwist;
using swerve_vehicle_control::PlanarVector;
using swerve_vehicle_control::WheelPosition;
}

TEST(SwerveKinematics, AppliesWheelOffsetInSteeringDirection)
{
  const WheelPosition pivot{0.55, 0.29};
  const auto forward = swerve_vehicle_control::wheel_center_position(pivot, 0.0, kOffset);
  EXPECT_NEAR(forward.x, 0.55 + kOffset, 1.0e-12);
  EXPECT_NEAR(forward.y, 0.29, 1.0e-12);

  const auto left = swerve_vehicle_control::wheel_center_position(pivot, kPi / 2.0, kOffset);
  EXPECT_NEAR(left.x, 0.55, 1.0e-12);
  EXPECT_NEAR(left.y, 0.29 + kOffset, 1.0e-12);
}

TEST(SwerveKinematics, OffsetChangesInverseVelocityAtWheelCenter)
{
  const WheelPosition pivot{0.55, 0.29};
  const auto center = swerve_vehicle_control::wheel_center_position(pivot, 0.0, kOffset);
  const auto velocity = swerve_vehicle_control::module_velocity(
    PlanarTwist{0.0, 0.0, 1.0}, center);
  EXPECT_NEAR(velocity.x, -0.29, 1.0e-12);
  EXPECT_NEAR(velocity.y, 0.55 + kOffset, 1.0e-12);
}

TEST(SwerveKinematics, GeneralLeastSquaresRecoversBodyTwist)
{
  const std::array<WheelPosition, 4> pivots{{
    {0.55, 0.29}, {0.55, -0.29}, {-0.55, 0.29}, {-0.55, -0.29}}};
  const std::array<double, 4> steering{{0.0, kPi / 2.0, kPi, -kPi / 2.0}};
  std::array<WheelPosition, 4> centers{};
  std::array<PlanarVector, 4> velocities{};
  const PlanarTwist expected{0.2, -0.1, 0.3};

  for (std::size_t i = 0; i < pivots.size(); ++i) {
    centers[i] = swerve_vehicle_control::wheel_center_position(
      pivots[i], steering[i], kOffset);
    velocities[i] = swerve_vehicle_control::module_velocity(expected, centers[i]);
  }

  const auto estimated =
    swerve_vehicle_control::estimate_body_twist(centers, velocities);
  EXPECT_NEAR(estimated.vx, expected.vx, 1.0e-12);
  EXPECT_NEAR(estimated.vy, expected.vy, 1.0e-12);
  EXPECT_NEAR(estimated.omega, expected.omega, 1.0e-12);
}
