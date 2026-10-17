#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/force_sensor_packet.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr float kServoCommandPeriodSeconds = 0.008F;
constexpr int kRpcPort = 8080;
constexpr auto kEndpointProbeTimeout = std::chrono::seconds(2);
constexpr auto kStateReadyTimeout = std::chrono::seconds(5);

struct TcpNoDelayResult
{
  int matching_sockets{0};
  int configured_sockets{0};
  int last_error{0};
};

double elapsedMilliseconds(const std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

bool tcpEndpointReachable(
  const std::string & ip, const int port, const std::chrono::milliseconds timeout,
  std::string & reason)
{
  const int socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0) {
    reason = std::string("socket: ") + std::strerror(errno);
    return false;
  }

  const int original_flags = ::fcntl(socket_fd, F_GETFL, 0);
  if (original_flags < 0 || ::fcntl(socket_fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
    reason = std::string("fcntl: ") + std::strerror(errno);
    ::close(socket_fd);
    return false;
  }

  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, ip.c_str(), &endpoint.sin_addr) != 1) {
    reason = "invalid IPv4 address";
    ::close(socket_fd);
    return false;
  }

  const int connect_result = ::connect(
    socket_fd, reinterpret_cast<const sockaddr *>(&endpoint), sizeof(endpoint));
  if (connect_result == 0) {
    ::close(socket_fd);
    return true;
  }
  if (errno != EINPROGRESS) {
    reason = std::string("connect: ") + std::strerror(errno);
    ::close(socket_fd);
    return false;
  }

  pollfd descriptor{socket_fd, POLLOUT, 0};
  const int poll_result = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  if (poll_result <= 0) {
    reason = poll_result == 0 ? "connect timeout" : std::string("poll: ") + std::strerror(errno);
    ::close(socket_fd);
    return false;
  }

  int socket_error = 0;
  socklen_t error_length = sizeof(socket_error);
  const bool ok = ::getsockopt(
    socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_length) == 0 && socket_error == 0;
  if (!ok) {
    reason = std::string("connect completion: ") +
      std::strerror(socket_error == 0 ? errno : socket_error);
  }
  ::close(socket_fd);
  return ok;
}

TcpNoDelayResult enableTcpNoDelayForPeer(const std::string & ip)
{
  TcpNoDelayResult result;
  in_addr expected_address{};
  if (::inet_pton(AF_INET, ip.c_str(), &expected_address) != 1) {
    result.last_error = EINVAL;
    return result;
  }
  DIR * descriptors = ::opendir("/proc/self/fd");
  if (descriptors == nullptr) {
    result.last_error = errno;
    return result;
  }
  while (dirent * entry = ::readdir(descriptors)) {
    char * end = nullptr;
    errno = 0;
    const long parsed = std::strtol(entry->d_name, &end, 10);
    if (errno != 0 || end == entry->d_name || *end != '\0' || parsed < 0 ||
      parsed > std::numeric_limits<int>::max())
    {
      continue;
    }
    const int socket_fd = static_cast<int>(parsed);
    sockaddr_storage peer{};
    socklen_t peer_length = sizeof(peer);
    if (::getpeername(socket_fd, reinterpret_cast<sockaddr *>(&peer), &peer_length) != 0 ||
      peer.ss_family != AF_INET)
    {
      continue;
    }
    const auto * ipv4 = reinterpret_cast<const sockaddr_in *>(&peer);
    if (ipv4->sin_addr.s_addr != expected_address.s_addr) {
      continue;
    }
    ++result.matching_sockets;
    const int enabled = 1;
    if (::setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0) {
      result.last_error = errno;
      continue;
    }
    int verified = 0;
    socklen_t verified_length = sizeof(verified);
    if (::getsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &verified, &verified_length) != 0 ||
      verified != 1)
    {
      result.last_error = errno != 0 ? errno : EIO;
      continue;
    }
    ++result.configured_sockets;
  }
  ::closedir(descriptors);
  return result;
}
}

namespace fairino_hardware
{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & sysinfo)
{
  if (hardware_interface::SystemInterface::on_init(sysinfo) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = sysinfo;  //info_是父类中定义的变量
  health_node_ = std::make_shared<rclcpp::Node>(
    "fairino_driver_health",
    rclcpp::NodeOptions().use_global_arguments(false));
  health_pub_ = health_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/fairino_driver/health", rclcpp::QoS(1).reliable());
  health_message_.data.resize(20);
  raw_force_pub_ = std::make_unique<WrenchPublisher>(
    health_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/fairino_driver/ft_raw", rclcpp::SensorDataQoS()));
  reference_force_pub_ = std::make_unique<WrenchPublisher>(
    health_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/fairino_driver/ft_reference", rclcpp::SensorDataQoS()));
  force_status_pub_ = std::make_unique<ForceStatusPublisher>(
    health_node_->create_publisher<fr10_motion_interfaces::msg::WrenchSourceStatus>(
      "/fairino_driver/ft_source_status", rclcpp::QoS(1).reliable()));
  const auto sensor_frame = info_.hardware_parameters.find("ft_sensor_frame");
  raw_force_pub_->msg_.header.frame_id = sensor_frame != info_.hardware_parameters.end() ?
    sensor_frame->second : "fairino_ft_sensor";
  const auto reference_frame = info_.hardware_parameters.find("ft_reference_frame");
  // RCS is not necessarily the sensor/TCP frame. Empty means unconfigured;
  // diagnostic data must not masquerade as a valid motion wrench frame.
  if (reference_frame != info_.hardware_parameters.end()) {
    reference_force_pub_->msg_.header.frame_id = reference_frame->second;
  }
  force_status_pub_->msg_.header.frame_id = raw_force_pub_->msg_.header.frame_id;
  force_status_pub_->msg_.source = "fairino_3.9.3_packet_raw";
  force_status_pub_->msg_.detail =
    "Raw sensor axes; reference data separate; active is not calibrated; "
    "packet observation stamp only, no independent FT timestamp/overload bit";

  static const std::array<std::string, 6> expected_joints{{"j1", "j2", "j3", "j4", "j5", "j6"}};
  if (info_.joints.size() != expected_joints.size()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exactly 6 FR10 joints are required; got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < expected_joints.size(); ++i) {
    if (info_.joints[i].name != expected_joints[i]) {
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint index %zu must be '%s' for FAIRINO SDK array mapping; got '%s'",
        i, expected_joints[i].c_str(), info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto robot_ip = info_.hardware_parameters.find("robot_ip");
  if (robot_ip != info_.hardware_parameters.end() && !robot_ip->second.empty()) {
    _controller_ip = robot_ip->second;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {

    //指令部分
    if (joint.command_interfaces.size() != 1) {    //开放servoJ
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
    //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    //关节状态部分
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "FairinoHardwareInterface"),
        "Joint '%s' have %s state interface as first state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces[2].name != hardware_interface::HW_IF_ACCELERATION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint '%s' state interfaces must be position, velocity, acceleration",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

  }
  return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init


std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &_jnt_torque_state[i]));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}


hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  using namespace std::chrono_literals;
  const auto logger = rclcpp::get_logger("FairinoHardwareInterface");
  RCLCPP_INFO(logger, "Starting deterministic FR10 activation");
  _servo_mode_started = false;
  _rpc_connected = false;
  _tcp_nodelay_verified = false;
  _first_write = true;
  // Error counters remain monotonic telemetry, while the last operation
  // result belongs to this activation session and must not poison a clean
  // reactivation forever.
  last_write_error_ = 0;
  //做变量的初始化工作
  servo_write_sequence_ = 0;
  servo_success_sequence_ = 0;
  last_servo_write_gap_s_ = 0.0;
  have_servo_write_time_ = false;
  if (!_ptr_robot) {
    _ptr_robot = std::make_unique<FRRobot>();    //创建机器人实例
  }
  for (int i = 0; i < 6; i++) {//初始化变量
    _jnt_position_command[i] = 0;
    _jnt_velocity_command[i] = 0;
    _jnt_torque_command[i] = 0;
    _jnt_position_state[i] = 0;
    _jnt_velocity_state[i] = 0;
    _jnt_torque_state[i] = 0;
  }
  _control_mode = 0;  //默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制

  std::string endpoint_error;
  const auto probe_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    logger, "Activation STEP 01 before TCP probe %s:%d", _controller_ip.c_str(),
    kRpcPort);
  if (!tcpEndpointReachable(
      _controller_ip, kRpcPort,
      std::chrono::duration_cast<std::chrono::milliseconds>(kEndpointProbeTimeout),
      endpoint_error))
  {
    RCLCPP_ERROR(
      logger, "Activation STEP 01 TCP probe failed elapsed_ms=%.3f: %s",
      elapsedMilliseconds(probe_start), endpoint_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger, "Activation STEP 01 after TCP probe rc=0 elapsed_ms=%.3f",
    elapsedMilliseconds(probe_start));

  const auto rpc_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "Activation STEP 02 before RPC");
  errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());  //建立xmlrpc连接
  RCLCPP_INFO(
    logger, "Activation STEP 02 after RPC rc=%d elapsed_ms=%.3f",
    returncode, elapsedMilliseconds(rpc_start));
  if (returncode != 0) {
    RCLCPP_ERROR(logger, "FR10 activation failed at STEP 02 RPC rc=%d", returncode);
    const auto close_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger, "Activation cleanup before CloseRPC");
    const errno_t close_error = _ptr_robot->CloseRPC();
    RCLCPP_INFO(
      logger, "Activation cleanup after CloseRPC rc=%d elapsed_ms=%.3f",
      close_error, elapsedMilliseconds(close_start));
    if (close_error != 0) {
      RCLCPP_ERROR(logger, "CloseRPC after RPC failure failed, error code: %d", close_error);
    }
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    _rpc_connected = true;
    RCLCPP_INFO(logger, "FAIRINO SDK connection established");
  }

  const auto close_after_failure = [this, &logger](const char * stage) {
      RCLCPP_ERROR(logger, "FR10 activation failed at %s", stage);
      errno_t end_error = 0;
      if (_servo_mode_started) {
        const auto end_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(logger, "Activation cleanup before ServoMoveEnd");
        end_error = _ptr_robot->ServoMoveEnd();
        RCLCPP_INFO(
          logger, "Activation cleanup after ServoMoveEnd rc=%d elapsed_ms=%.3f",
          end_error, elapsedMilliseconds(end_start));
      }
      const auto close_start = std::chrono::steady_clock::now();
      RCLCPP_INFO(logger, "Activation cleanup before CloseRPC");
      const errno_t close_error = _ptr_robot->CloseRPC();
      RCLCPP_INFO(
        logger, "Activation cleanup after CloseRPC rc=%d elapsed_ms=%.3f",
        close_error, elapsedMilliseconds(close_start));
      _servo_mode_started = false;
      _rpc_connected = false;
      _tcp_nodelay_verified = false;
      if (end_error != 0 || close_error != 0) {
        RCLCPP_ERROR(
          logger, "Activation cleanup returned ServoMoveEnd=%d CloseRPC=%d",
          end_error, close_error);
      }
      return hardware_interface::CallbackReturn::ERROR;
    };

  const TcpNoDelayResult tcp_nodelay = enableTcpNoDelayForPeer(_controller_ip);
  RCLCPP_INFO(
    logger,
    "Activation STEP 02A TCP_NODELAY matching=%d configured=%d last_error=%d",
    tcp_nodelay.matching_sockets, tcp_nodelay.configured_sockets,
    tcp_nodelay.last_error);
  if (tcp_nodelay.matching_sockets == 0 ||
    tcp_nodelay.configured_sockets != tcp_nodelay.matching_sockets)
  {
    return close_after_failure("STEP 02A TCP_NODELAY verification");
  }
  _tcp_nodelay_verified = true;

  int sdk_com_state = -1;
  const auto com_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "Activation STEP 03 before GetSDKComState");
  returncode = _ptr_robot->GetSDKComState(&sdk_com_state);
  RCLCPP_INFO(
    logger, "Activation STEP 03 after GetSDKComState rc=%d state=%d elapsed_ms=%.3f",
    returncode, sdk_com_state, elapsedMilliseconds(com_start));
  if (returncode != 0 || sdk_com_state != 0) {
    return close_after_failure("STEP 03 GetSDKComState");
  }

  ROBOT_STATE_PKG robot_state{};
  const auto state_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "Activation STEP 04 before GetRobotRealTimeState");
  returncode = _ptr_robot->GetRobotRealTimeState(&robot_state);
  RCLCPP_INFO(
    logger,
    "Activation STEP 04 after GetRobotRealTimeState rc=%d elapsed_ms=%.3f "
    "enable=%d mode=%u robot_state=%u program_state=%u error=%d/%d emergency=%u safety=%u/%u",
    returncode, elapsedMilliseconds(state_start), robot_state.rbtEnableState,
    robot_state.robot_mode, robot_state.robot_state, robot_state.program_state,
    robot_state.main_code, robot_state.sub_code, robot_state.EmergencyStop,
    robot_state.safety_stop0_state, robot_state.safety_stop1_state);

  if (returncode != 0) {
    return close_after_failure("STEP 04 GetRobotRealTimeState");
  }

  // Alarms require operator diagnosis/reset. Never clear one as a side
  // effect of bringing up a ros2_control hardware interface.

  if (returncode != 0 || robot_state.main_code != 0 || robot_state.sub_code != 0 ||
    robot_state.EmergencyStop != 0 || robot_state.safety_stop0_state != 0 ||
    robot_state.safety_stop1_state != 0 ||
    robot_state.robot_state == 2 || robot_state.program_state == 2)
  {
    return close_after_failure("STEP 04 unsafe/not-ready robot state");
  }

  if (robot_state.robot_mode != 0) {
    RCLCPP_ERROR(
      logger,
      "Robot is not in Auto mode. Operator must select Auto before hardware activation; Mode(0) is not called automatically.");
    return close_after_failure("STEP 04 operator Auto mode required");
  }

  if (robot_state.rbtEnableState == 0) {
    RCLCPP_ERROR(
      logger,
      "Robot is disabled. Operator must enable it before hardware activation; RobotEnable(1) is not called automatically.");
    return close_after_failure("STEP 05 operator enable required");
  }
  RCLCPP_INFO(logger, "Activation STEP 05 operator-controlled enable verified");

  //做第一步的工作，读取当前状态数据
  JointPos jntpos{};
  const auto joint_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "Activation STEP 06 before bounded GetActualJointPosDegree(flag=1) polling");
  const auto joint_deadline = joint_start + kStateReadyTimeout;
  do {
    returncode = _ptr_robot->GetActualJointPosDegree(1, &jntpos);
    if (returncode == 0) {
      break;
    }
    rclcpp::sleep_for(50ms);
  } while (std::chrono::steady_clock::now() < joint_deadline);
  RCLCPP_INFO(
    logger, "Activation STEP 06 after GetActualJointPosDegree rc=%d elapsed_ms=%.3f",
    returncode, elapsedMilliseconds(joint_start));
  /*
  获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
  因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
  */
  if (returncode == 0) {
    for (int j = 0; j < 6; j++) {
      _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
      _jnt_position_state[j] = _jnt_position_command[j];
      _jnt_velocity_state[j] = health_state_.actual_qd[j] / 180.0 * M_PI;
      _jnt_torque_state[j] = health_state_.actual_qdd[j] / 180.0 * M_PI;
      _previous_command_deg[j] = jntpos.jPos[j];
    }
    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Activation actual SDK deg: [%.6f %.6f %.6f %.6f %.6f %.6f]",
      jntpos.jPos[0], jntpos.jPos[1], jntpos.jPos[2], jntpos.jPos[3],
      jntpos.jPos[4], jntpos.jPos[5]);
    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Activation initial command ROS rad: [%.6f %.6f %.6f %.6f %.6f %.6f]",
      _jnt_position_command[0], _jnt_position_command[1], _jnt_position_command[2],
      _jnt_position_command[3], _jnt_position_command[4], _jnt_position_command[5]);
    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Activation command-actual deg: [0 0 0 0 0 0]");

    const auto servo_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger, "Activation STEP 07 before ServoMoveStart");
    returncode = _ptr_robot->ServoMoveStart();
    RCLCPP_INFO(
      logger, "Activation STEP 07 after ServoMoveStart rc=%d elapsed_ms=%.3f",
      returncode, elapsedMilliseconds(servo_start));
    if (returncode != 0) {
      RCLCPP_ERROR(logger, "ServoMoveStart failed, error code: %d", returncode);
      return close_after_failure("STEP 07 ServoMoveStart");
    }
    _servo_mode_started = true;

    const TcpNoDelayResult servo_tcp_nodelay =
      enableTcpNoDelayForPeer(_controller_ip);
    RCLCPP_INFO(
      logger,
      "Activation STEP 08A TCP_NODELAY matching=%d configured=%d last_error=%d",
      servo_tcp_nodelay.matching_sockets,
      servo_tcp_nodelay.configured_sockets,
      servo_tcp_nodelay.last_error);
    if (servo_tcp_nodelay.matching_sockets == 0 ||
      servo_tcp_nodelay.configured_sockets !=
      servo_tcp_nodelay.matching_sockets)
    {
      return close_after_failure("STEP 08A TCP_NODELAY verification");
    }
    ExaxisPos extcmd{0, 0, 0, 0};
    const auto hold_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger, "Activation STEP 08 before initial hold ServoJ");
    const auto servo_write_time = std::chrono::steady_clock::now();
    if (have_servo_write_time_) {
      last_servo_write_gap_s_ =
        std::chrono::duration<double>(
        servo_write_time - last_servo_write_time_).count();
    }
    last_servo_write_time_ = servo_write_time;
    have_servo_write_time_ = true;
    ++servo_write_sequence_;
    returncode = _ptr_robot->ServoJ(
      &jntpos, &extcmd, 0, 0, kServoCommandPeriodSeconds, 0, 0);
    if (returncode == 0) {
      servo_success_sequence_ = servo_write_sequence_;
    }
    RCLCPP_INFO(
      logger, "Activation STEP 08 after initial hold ServoJ rc=%d elapsed_ms=%.3f",
      returncode, elapsedMilliseconds(hold_start));
    if (returncode != 0) {
      RCLCPP_ERROR(logger, "Initial hold ServoJ failed, error code: %d", returncode);
      return close_after_failure("STEP 08 initial hold ServoJ");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "ServoMoveStart and initial hold ServoJ succeeded; hardware ACTIVE");
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(
      logger,
      "GetActualJointPosDegree readiness failed, last error code: %d", returncode);
    return close_after_failure("STEP 06 GetActualJointPosDegree readiness timeout");
  }
}


hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
  return stopAndClose("deactivate");
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("FairinoHardwareInterface"),
    "Hardware entered ERROR; stopping Servo mode and closing RPC");
  return stopAndClose("error");
}

hardware_interface::CallbackReturn FairinoHardwareInterface::stopAndClose(const char * context)
{
  if (!_ptr_robot) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  errno_t end_error = 0;
  if (_servo_mode_started) {
    end_error = _ptr_robot->ServoMoveEnd();
  }
  const errno_t close_error = _rpc_connected ? _ptr_robot->CloseRPC() : 0;
  _servo_mode_started = false;
  _rpc_connected = false;
  _tcp_nodelay_verified = false;
  if (end_error != 0 || close_error != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Hardware %s cleanup returned: ServoMoveEnd=%d CloseRPC=%d",
      context, end_error, close_error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("FairinoHardwareInterface"),
    "System successfully stopped (%s)", context);
  return hardware_interface::CallbackReturn::SUCCESS;
}


void FairinoHardwareInterface::publishHealth(int read_error)
{
  auto & data = health_message_.data;
  data[0] = 2;   // schema v2 extends v1 with per-ServoJ transport telemetry
  data[1] = static_cast<double>(health_sequence_);
  data[2] = health_state_.frame_cnt;
  data[3] = std::chrono::duration<double>(last_frame_time_.time_since_epoch()).count();
  data[4] = read_error;
  data[5] = last_write_error_;
  data[6] = health_state_.rbtEnableState;
  data[7] = health_state_.robot_mode;
  data[8] = health_state_.EmergencyStop;
  data[9] = health_state_.collisionState;
  data[10] = health_state_.main_code;
  data[11] = health_state_.sub_code;
  data[12] = health_state_.safety_stop0_state;
  data[13] = health_state_.safety_stop1_state;
  data[14] = static_cast<double>(read_errors_);
  data[15] = static_cast<double>(write_errors_);
  data[16] = static_cast<double>(servo_write_sequence_);
  data[17] = static_cast<double>(servo_success_sequence_);
  data[18] = last_servo_write_gap_s_;
  data[19] = last_write_error_;
  health_pub_->publish(health_message_);
}

void FairinoHardwareInterface::publishForceSensor(const rclcpp::Time & time, bool packet_online)
{
  const auto ft = decodeForceSensorPacket(health_state_);
  if (force_status_pub_->trylock()) {
    auto & status = force_status_pub_->msg_;
    status.header.stamp = time;
    status.sequence = health_sequence_;
    status.online = packet_online && ft.active && ft.raw_finite;
    status.calibrated = false;
    status.overload = false;  // Not reported by the v3.9.3 packet; never fabricated.
    status.receipt_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_frame_time_).count();
    force_status_pub_->unlockAndPublish();
  }
  if (!packet_online || !ft.active || health_sequence_ == force_published_sequence_) {
    return;
  }
  force_published_sequence_ = health_sequence_;
  const auto publish = [&time](WrenchPublisher & publisher, const std::array<double, 6> & values) {
      if (!publisher.trylock()) {return;}
      auto & message = publisher.msg_;
      message.header.stamp = time;
      message.wrench.force.x = values[0];
      message.wrench.force.y = values[1];
      message.wrench.force.z = values[2];
      message.wrench.torque.x = values[3];
      message.wrench.torque.y = values[4];
      message.wrench.torque.z = values[5];
      publisher.unlockAndPublish();
    };
  publish(*raw_force_pub_, ft.raw);
  publish(*reference_force_pub_, ft.reference);
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
  const int state_error = _ptr_robot->GetRobotRealTimeState(&health_state_);
  const auto tick = std::chrono::steady_clock::now();
  if (state_error == 0 && health_state_.frame_cnt != last_frame_) {
    last_frame_ = health_state_.frame_cnt;
    last_frame_time_ = tick;
    ++health_sequence_;
  }
  const bool stale = last_frame_<0 ||
      std::chrono::duration<double>(tick - last_frame_time_).count()>0.050;
  if (state_error != 0 || stale || health_state_.EmergencyStop || health_state_.collisionState ||
    health_state_.main_code || health_state_.sub_code || health_state_.safety_stop0_state ||
    health_state_.safety_stop1_state || health_state_.robot_mode != 0 ||
    !health_state_.rbtEnableState)
  {
    ++read_errors_;
    publishForceSensor(time, false);
    publishHealth(state_error != 0 ? state_error : -1);
    return hardware_interface::return_type::ERROR;
  }
  // One SDK real-time frame is the timestamp authority for q, qd and qdd.
  // Avoiding a second RPC per cycle also removes avoidable ServoJ jitter.
  for (int i = 0; i < 6; ++i) {
    _jnt_position_state[i] = health_state_.jt_cur_pos[i] / 180.0 * M_PI;
    _jnt_velocity_state[i] = health_state_.actual_qd[i] / 180.0 * M_PI;
    _jnt_torque_state[i] = health_state_.actual_qdd[i] / 180.0 * M_PI;
    if (!std::isfinite(_jnt_position_state[i]) ||
      !std::isfinite(_jnt_velocity_state[i]) ||
      !std::isfinite(_jnt_torque_state[i]))
    {
      ++read_errors_;
      publishHealth(-2);
      return hardware_interface::return_type::ERROR;
    }
  }

  //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
  //     _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  publishForceSensor(time, true);
  publishHealth(0);
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (_control_mode == 0) {//位置控制模式
    if (std::any_of(
        &_jnt_position_command[0], &_jnt_position_command[6], \
        [](double c) {return not std::isfinite(c);}))
    {
      return hardware_interface::return_type::ERROR;
    }
    JointPos cmd;
    ExaxisPos extcmd{0, 0, 0, 0};
    for (auto j = 0; j < 6; j++) {
      cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180;   //注意单位转换
    }
    if (_first_write) {
      RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "First write ROS rad: [%.6f %.6f %.6f %.6f %.6f %.6f]",
        _jnt_position_command[0], _jnt_position_command[1], _jnt_position_command[2],
        _jnt_position_command[3], _jnt_position_command[4], _jnt_position_command[5]);
      RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "First write SDK deg: [%.6f %.6f %.6f %.6f %.6f %.6f], actual deg: "
        "[%.6f %.6f %.6f %.6f %.6f %.6f], period=%.6f s cmdT=%.6f s",
        cmd.jPos[0], cmd.jPos[1], cmd.jPos[2], cmd.jPos[3], cmd.jPos[4], cmd.jPos[5],
        _jnt_position_state[0] / M_PI * 180.0, _jnt_position_state[1] / M_PI * 180.0,
        _jnt_position_state[2] / M_PI * 180.0, _jnt_position_state[3] / M_PI * 180.0,
        _jnt_position_state[4] / M_PI * 180.0, _jnt_position_state[5] / M_PI * 180.0,
        period.seconds(), kServoCommandPeriodSeconds);
      _first_write = false;
    }
    const errno_t returncode = _ptr_robot->ServoJ(
      &cmd, &extcmd, 0, 0, kServoCommandPeriodSeconds, 0, 0);
    if (returncode != 0) {
      last_write_error_ = returncode;
      ++write_errors_;
      publishHealth(0);
      int main_error = -1;
      int sub_error = -1;
      const errno_t error_query =
        _ptr_robot->GetRobotErrorCode(&main_error, &sub_error);
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("FairinoHardwareInterface"), _steady_clock, 1000,
        "ServoJ failed code=%d actual_deg=[%.6f %.6f %.6f %.6f %.6f %.6f] "
        "command_deg=[%.6f %.6f %.6f %.6f %.6f %.6f] "
        "previous_deg=[%.6f %.6f %.6f %.6f %.6f %.6f] period=%.6f cmdT=%.6f "
        "GetRobotErrorCode=%d main=%d sub=%d",
        returncode,
        _jnt_position_state[0] / M_PI * 180.0, _jnt_position_state[1] / M_PI * 180.0,
        _jnt_position_state[2] / M_PI * 180.0, _jnt_position_state[3] / M_PI * 180.0,
        _jnt_position_state[4] / M_PI * 180.0, _jnt_position_state[5] / M_PI * 180.0,
        cmd.jPos[0], cmd.jPos[1], cmd.jPos[2], cmd.jPos[3], cmd.jPos[4], cmd.jPos[5],
        _previous_command_deg[0], _previous_command_deg[1], _previous_command_deg[2],
        _previous_command_deg[3], _previous_command_deg[4], _previous_command_deg[5],
        period.seconds(), kServoCommandPeriodSeconds,
        error_query, main_error, sub_error);
      (void)stopAndClose("ServoJ failure");
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t j = 0; j < _previous_command_deg.size(); ++j) {
      _previous_command_deg[j] = cmd.jPos[j];
    }
  } else if (_control_mode == 1) {//扭矩控制模式
    if (std::any_of(
        &_jnt_torque_command[0], &_jnt_torque_command[6], \
        [](double c) {return not std::isfinite(c);}))
    {
      return hardware_interface::return_type::ERROR;
    }
    //_ptr_robot->write(_jnt_torque_command);//注意单位转换
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface,
  hardware_interface::SystemInterface)
