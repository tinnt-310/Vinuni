#ifndef SWERVE_VEHICLE_CONTROL__SWERVE_KINEMATICS_HPP_
#define SWERVE_VEHICLE_CONTROL__SWERVE_KINEMATICS_HPP_

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/QR>

namespace swerve_vehicle_control
{

struct WheelPosition
{
  double x;
  double y;
};

struct PlanarVector
{
  double x;
  double y;
};

struct PlanarTwist
{
  double vx;
  double vy;
  double omega;
};

inline WheelPosition wheel_center_position(
  const WheelPosition & steering_pivot, double steering_angle, double wheel_offset_x)
{
  return {
    steering_pivot.x + wheel_offset_x * std::cos(steering_angle),
    steering_pivot.y + wheel_offset_x * std::sin(steering_angle)};
}

inline PlanarVector module_velocity(
  const PlanarTwist & body_twist, const WheelPosition & wheel_center)
{
  return {
    body_twist.vx - body_twist.omega * wheel_center.y,
    body_twist.vy + body_twist.omega * wheel_center.x};
}

inline PlanarTwist estimate_body_twist(
  const std::array<WheelPosition, 4> & wheel_centers,
  const std::array<PlanarVector, 4> & wheel_velocities)
{
  Eigen::Matrix<double, 8, 3> system;
  Eigen::Matrix<double, 8, 1> observations;
  for (std::size_t i = 0; i < wheel_centers.size(); ++i) {
    system.row(2 * i) << 1.0, 0.0, -wheel_centers[i].y;
    system.row(2 * i + 1) << 0.0, 1.0, wheel_centers[i].x;
    observations(2 * i) = wheel_velocities[i].x;
    observations(2 * i + 1) = wheel_velocities[i].y;
  }
  const Eigen::Matrix<double, 3, 1> solution =
    system.colPivHouseholderQr().solve(observations);
  return {solution(0), solution(1), solution(2)};
}

}  // namespace swerve_vehicle_control

#endif  // SWERVE_VEHICLE_CONTROL__SWERVE_KINEMATICS_HPP_
