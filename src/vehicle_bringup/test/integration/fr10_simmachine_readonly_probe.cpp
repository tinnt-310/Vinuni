// Read-only SDK 3.9.3 probe. Does not enable, change Mode, reset or move.
// Run only after Docker inspection proves the IP belongs to SimMachine.
#include "robot.h"
#include <chrono>
#include <thread>
#include <iomanip>
#include <limits>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <cstring>

int main(int argc, char **argv) {
  double soak_s = 0.;
  if (argc != 2) {
    if (argc != 4 || std::strcmp(argv[2], "--soak-seconds")) return 2;
    char *end{};
    soak_s = std::strtod(argv[3], &end);
    if (!end || *end || !std::isfinite(soak_s) || soak_s <= 0. || soak_s > 60.) return 2;
  }
  FRRobot robot;
  const int rpc = robot.RPC(argv[1]);
  char sdk[128]{};
  char model[64]{}, web[64]{}, controller[64]{};
  int sdk_rc = robot.GetSDKVersion(sdk);
  int version_rc = rpc == 0 ? robot.GetSoftwareVersion(model, web, controller) : -1;
  ROBOT_STATE_PKG state{};
  int state_rc = -1;
  if (rpc == 0) {
    for (int i=0; i<30; ++i) {
      state_rc = robot.GetRobotRealTimeState(&state);
      if (state_rc == 0 && state.frame_head == 0x5A5A) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  std::size_t soak_reads = 0, fresh_frames = 0, read_errors = 0, unhealthy_frames = 0;
  double max_frame_age_s = 0., max_read_us = 0.;
  const auto initial_mode = state.robot_mode;
  const auto initial_enabled = state.rbtEnableState;
  bool state_unchanged = true;
  if (soak_s > 0. && state_rc == 0) {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    auto next = started, last_new_frame = started;
    auto frame = state.frame_cnt;
    while (std::chrono::duration<double>(Clock::now()-started).count() < soak_s) {
      const auto before = Clock::now();
      const int rc = robot.GetRobotRealTimeState(&state);
      const auto now = Clock::now();
      ++soak_reads;
      max_read_us = std::max(max_read_us,
          std::chrono::duration<double,std::micro>(now-before).count());
      if (rc != 0 || state.frame_head != 0x5A5A) ++read_errors;
      else {
        if (state.frame_cnt != frame) {
          frame = state.frame_cnt; last_new_frame = now; ++fresh_frames;
        }
        bool finite_joints = true;
        for (double value : state.jt_cur_pos) finite_joints &= std::isfinite(value);
        if (!finite_joints || state.main_code || state.sub_code || state.EmergencyStop ||
            state.safety_stop0_state || state.safety_stop1_state) ++unhealthy_frames;
        state_unchanged &= state.robot_mode == initial_mode && state.rbtEnableState == initial_enabled;
      }
      max_frame_age_s = std::max(max_frame_age_s,
          std::chrono::duration<double>(now-last_new_frame).count());
      next += std::chrono::milliseconds(8);
      std::this_thread::sleep_until(next);
    }
  }
  const bool soak_pass = soak_s > 0. && soak_reads >= .9*125.*soak_s &&
      fresh_frames > 0 && read_errors == 0 && unhealthy_frames == 0 &&
      state_unchanged && max_frame_age_s <= .05;
  // Preserve measured joint precision for the independent CAD checks.
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "{\"rpc_rc\":" << rpc << ",\"sdk_rc\":" << sdk_rc
            << ",\"sdk\":" << std::quoted(sdk)
            << ",\"version_rc\":" << version_rc
            << ",\"model\":" << std::quoted(model)
            << ",\"web_version\":" << std::quoted(web)
            << ",\"controller_version\":" << std::quoted(controller)
            << ",\"state_rc\":" << state_rc
            << ",\"frame_head\":" << state.frame_head
            << ",\"mode\":" << static_cast<int>(state.robot_mode)
            << ",\"enabled\":" << state.rbtEnableState
            << ",\"main_error\":" << state.main_code
            << ",\"sub_error\":" << state.sub_code
            << ",\"emergency\":" << static_cast<int>(state.EmergencyStop)
            << ",\"safety0\":" << static_cast<int>(state.safety_stop0_state)
            << ",\"safety1\":" << static_cast<int>(state.safety_stop1_state)
            << ",\"joint_position_deg\":[";
  for (int i=0; i<6; ++i) std::cout << (i ? "," : "") << state.jt_cur_pos[i];
  std::cout << "],\"ft_sensor_active\":" << static_cast<int>(state.ft_sensor_active)
            << ",\"ft_sensor_raw_data\":[";
  bool raw_finite=true, reference_finite=true;
  const auto number = [](double value) {
    if (std::isfinite(value)) std::cout << value;
    else std::cout << "null";
  };
  for (int i=0; i<6; ++i) {
    std::cout << (i ? "," : ""); number(state.ft_sensor_raw_data[i]);
    raw_finite &= std::isfinite(state.ft_sensor_raw_data[i]);
  }
  std::cout << "],\"ft_sensor_data\":[";
  for (int i=0; i<6; ++i) {
    std::cout << (i ? "," : ""); number(state.ft_sensor_data[i]);
    reference_finite &= std::isfinite(state.ft_sensor_data[i]);
  }
  std::cout << "],\"ft_raw_finite\":" << (raw_finite ? "true" : "false")
            << ",\"ft_reference_finite\":" << (reference_finite ? "true" : "false")
            << ",\"read_only\":true,\"actuator_commands_sent\":0"
            << ",\"state_read_soak_seconds\":" << soak_s
            << ",\"state_read_soak_calls\":" << soak_reads
            << ",\"state_read_soak_fresh_frames\":" << fresh_frames
            << ",\"state_read_soak_errors\":" << read_errors
            << ",\"state_read_soak_unhealthy_frames\":" << unhealthy_frames
            << ",\"state_read_soak_mode_enable_unchanged\":" << (state_unchanged ? "true" : "false")
            << ",\"state_read_soak_max_frame_age_s\":" << max_frame_age_s
            << ",\"state_read_soak_max_read_us\":" << max_read_us
            << ",\"state_read_soak_pass\":" << (soak_pass ? "true" : "false") << "}" << std::endl;
  robot.CloseRPC();
  return rpc == 0 && version_rc == 0 && state_rc == 0 ? 0 : 1;
}
