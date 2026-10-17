#include <iostream>
#include <limits>
#include "libfairino/include/robot_types.h"
#include "fairino_hardware/force_sensor_packet.hpp"

int main()
{
  ROBOT_STATE_PKG packet{};
  bool ok = true;
  const auto check = [&ok](bool value, const char * name) {
      if (!value) {std::cerr << "FAIL " << name << '\n'; ok = false;}
    };
  auto ft = fairino_hardware::decodeForceSensorPacket(packet);
  check(!ft.active && ft.raw_finite, "inactive zeros are not online calibrated FT");
  packet.ft_sensor_active = 1;
  for (unsigned i = 0; i < 6; ++i) {
    packet.ft_sensor_raw_data[i] = 10. + i;
    packet.ft_sensor_data[i] = -20. - i;
  }
  ft = fairino_hardware::decodeForceSensorPacket(packet);
  check(ft.active && ft.raw_finite && ft.reference_finite, "active finite packet");
  check(ft.raw[0] == 10. && ft.raw[3] == 13., "raw SI components preserved");
  check(ft.reference[0] == -20. && ft.reference[3] == -23., "reference not substituted for raw");
  packet.ft_sensor_data[2] = std::numeric_limits<double>::quiet_NaN();
  ft = fairino_hardware::decodeForceSensorPacket(packet);
  check(ft.raw_finite && !ft.reference_finite, "reference validity independent of raw");
  packet.ft_sensor_raw_data[1] = std::numeric_limits<double>::infinity();
  ft = fairino_hardware::decodeForceSensorPacket(packet);
  check(!ft.raw_finite, "nonfinite raw rejected");
  packet.ft_sensor_active = 2;
  check(
    !fairino_hardware::decodeForceSensorPacket(packet).active,
    "unknown activation fails closed");
  return ok ? 0 : 1;
}
