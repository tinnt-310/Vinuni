#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace fairino_hardware
{

// A packet frame counter is a transport observation, not an independent FT
// sample clock or a sensor calibration certificate. Never infer activation
// from six zero readings: an inactive sensor also reports zeros.
struct ForceSensorPacket
{
  std::array<double, 6> raw{};
  std::array<double, 6> reference{};
  bool active{false};
  bool raw_finite{false};
  bool reference_finite{false};
};

template<typename Packet>
ForceSensorPacket decodeForceSensorPacket(const Packet & packet)
{
  ForceSensorPacket result;
  std::copy_n(packet.ft_sensor_raw_data, 6, result.raw.begin());
  std::copy_n(packet.ft_sensor_data, 6, result.reference.begin());
  result.active = packet.ft_sensor_active == 1;
  const auto finite = [](double value) {return std::isfinite(value);};
  result.raw_finite = std::all_of(result.raw.begin(), result.raw.end(), finite);
  result.reference_finite = std::all_of(result.reference.begin(), result.reference.end(), finite);
  return result;
}

}  // namespace fairino_hardware
