#include "robot.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <thread>

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void on_signal(int) { interrupted = 1; }
static const std::set<std::string> cases{
  "state", "ft_read", "servoj", "servocart_absolute", "servocart_base",
  "servocart_tool", "servojt", "servojt_limits", "ft_control_basic",
  "ft_control_mb", "ft_control_extended", "ft_compliance", "ft_spiral",
  "ft_linear", "ft_rotational", "ft_surface", "ft_zero",
  "ft_payload_record", "ft_payload_compute", "ft_movel"};

static void require(bool ok, const std::string &why) {
  if (!ok) throw std::runtime_error(why);
}
static double number(const json &o, const char *key, double low, double high) {
  require(o.contains(key) && o.at(key).is_number(), std::string("Missing numeric ") + key);
  const double x = o.at(key).get<double>();
  require(std::isfinite(x) && x >= low && x <= high, std::string("Out of range: ") + key);
  return x;
}
static int integer(const json &o, const char *key, int low, int high) {
  const double x = number(o, key, low, high);
  require(std::floor(x) == x, std::string("Expected integer: ") + key);
  return static_cast<int>(x);
}
template<std::size_t N> static std::array<double, N> array(
    const json &o, const char *key, double low, double high) {
  require(o.contains(key) && o.at(key).is_array() && o.at(key).size() == N,
          std::string("Expected array length ") + std::to_string(N) + ": " + key);
  std::array<double, N> a{};
  for (std::size_t i = 0; i < N; ++i) {
    require(o.at(key)[i].is_number(), std::string("Non-numeric array: ") + key);
    a[i] = o.at(key)[i].get<double>();
    require(std::isfinite(a[i]) && a[i] >= low && a[i] <= high,
            std::string("Array value out of range: ") + key);
  }
  return a;
}
static bool is_motion(const std::string &s) {
  return s != "state" && s != "ft_read" && s != "ft_zero" &&
         s != "ft_payload_record" && s != "ft_payload_compute";
}
static bool is_force(const std::string &s) {
  return s.rfind("ft_", 0) == 0 && s != "ft_read" && s != "ft_zero" &&
         s != "ft_payload_record" && s != "ft_payload_compute";
}

static void validate(const json &c, const std::string &s) {
  require(cases.count(s), "Unknown case: " + s);
  require(c.at("target") == "real" || c.at("target") == "sim", "target must be real or sim");
  for (const char *key : {"ip", "expected_web_version", "expected_sdk_contains", "expected_model_prefix"})
    require(c.at(key).is_string() && !c.at(key).get<std::string>().empty(), std::string("Missing ") + key);
  in_addr ip{};
  require(inet_pton(AF_INET,c.at("ip").get<std::string>().c_str(),&ip)==1,"ip must be an IPv4 address");
  number(c, "duration_s", 0.04, 10);
  number(c, "period_s", 0.001, 0.008);
  const auto &g = c.at("guards");
  number(g, "max_frame_age_s", 0.008, 0.2);
  for (const char *key : {"max_joint_delta_deg", "max_tcp_delta_mm", "max_rotation_deg", "max_force_n", "max_torque_nm"})
    number(g, key, 0.001, 10000);
  if (s.rfind("servo", 0) == 0 && s.rfind("servojt", 0) != 0) {
    const auto &v = c.at("servo");
    integer(v, "joint_index", 0, 5); integer(v, "cart_axis", 0, 2);
    number(v, "joint_delta_deg", -1, 1); number(v, "cart_delta_mm", -50, 50);
    number(v, "minimum_joint_response_deg", 0.001, 1);
    number(v, "minimum_cart_response_mm", 0.001, 2);
    number(v, "final_joint_tolerance_deg", 0.001, 1);
    number(v, "final_cart_tolerance_mm", 0.001, 2);
    const auto ext = array<4>(v, "ext_axis_mm", -100000, 100000);
    require(std::all_of(ext.begin(), ext.end(), [](double x) { return x == 0; }),
            "This FR10 bench only supports zero/unused external axes");
    require(std::abs(v.at("joint_delta_deg").get<double>()) > v.at("minimum_joint_response_deg").get<double>(),
            "joint_delta_deg must exceed minimum_joint_response_deg");
    require(std::abs(v.at("cart_delta_mm").get<double>()) > v.at("minimum_cart_response_mm").get<double>(),
            "cart_delta_mm must exceed minimum_cart_response_mm");
    require(std::abs(v.at("joint_delta_deg").get<double>()) < g.at("max_joint_delta_deg").get<double>() &&
            std::abs(v.at("cart_delta_mm").get<double>()) < g.at("max_tcp_delta_mm").get<double>(),
            "Servo target must be inside the observation envelope");
  }
  if (s.rfind("servojt", 0) == 0) {
    const auto &t = c.at("torque");
    require(t.at("prepared") == true, "torque.prepared must be true for a commissioned torque fixture");
    const auto torques = array<6>(t, "torque_nm", -500, 500);
    const auto limits = array<6>(t, "max_abs_torque_nm", 0.001, 500);
    for (int i = 0; i < 6; ++i) require(std::abs(torques[i]) <= limits[i], "Torque exceeds configured ceiling");
    array<6>(t, "power_limit_w", 0.001, 5000);
    array<6>(t, "velocity_limit_deg_s", 0.001, 10);
  }
  if (is_force(s)) {
    const auto &f = c.at("force");
    require(f.at("fixture_ready") == true, "force.fixture_ready must be true for the prepared contact fixture");
    integer(f, "sensor_id", 1, 14);
    const auto select = array<6>(f, "select", 0, 1);
    require(std::any_of(select.begin(), select.end(), [](double x) { return x == 1; }), "Select at least one force axis");
    for (double x : select) require(x == 0 || x == 1, "force.select must contain 0 or 1");
    const auto wrench = array<6>(f, "desired_wrench", -1000, 1000);
    array<6>(f, "pid", 0, 1);
    for (int i = 0; i < 6; ++i) require(std::abs(wrench[i]) <= g.at(i < 3 ? "max_force_n" : "max_torque_nm").get<double>(), "Desired wrench exceeds observation ceiling");
    number(f, "max_dis_mm", 0.001, g.at("max_tcp_delta_mm").get<double>());
    number(f, "max_ang_deg", 0.001, g.at("max_rotation_deg").get<double>());
    array<2>(f, "mass_rx_ry", 0.1, 10); array<2>(f, "damping_rx_ry", 0.1, 50);
    array<2>(f, "threshold_rx_ry", 0, 10); array<2>(f, "adjust_coeff_rx_ry", 0, 1);
    integer(f, "filter", 0, 1); integer(f, "position_adaptation", 0, 1);
    integer(f, "rcs", 0, 1); number(f, "threshold_n", 0.01, 100);
    number(f, "speed_mm_s", 0.001, 1);
    number(f, "max_distance_mm", 0.001, g.at("max_tcp_delta_mm").get<double>());
    number(f, "spiral_dr_mm", 0.001, 1); number(f, "spiral_max_time_ms", 1, 5000);
    number(f, "rotation_speed_deg_s", 0.001, 1);
    number(f, "max_angle_deg", 0.001, g.at("max_rotation_deg").get<double>());
    integer(f, "rotation_force_direction", 1, 2); integer(f, "rotation_direction", 1, 2);
    integer(f, "linear_direction", 0, 1); integer(f, "surface_direction", 1, 2);
    integer(f, "surface_axis", 1, 3);
    if (s == "ft_compliance") {
      number(f, "compliance_p", 0.000001, 1); number(f, "compliance_threshold_n", 0.01, 100);
    }
  }
  if (s == "ft_zero") {
    require(c.at("calibration").at("stationary_no_contact_confirmed") == true,
            "Confirm stationary no-contact tare conditions in calibration config");
    integer(c.at("calibration"), "zero_action", 0, 1);
  }
  if (s == "ft_payload_record") {
    require(c.at("calibration").at("payload_record_pose_prepared") == true, "Prepare the vendor payload-record pose first");
    integer(c.at("calibration"), "sensor_coordinate_id", 1, 14);
  }
  if (s == "ft_payload_compute")
    require(c.at("calibration").at("payload_record_completed") == true, "Complete the payload record procedure first");
  if (s == "ft_movel") {
    const auto &m=c.at("force_movel"), &f=c.at("force");
    require(m.at("reference_verified")==true,
      "Verify sensor_id and feedback Fz refer to the prepared tool-Z force frame");
    integer(m,"expected_tool",0,14);
    integer(m,"tool_axis",0,1); // tangential X/Y only; never search for contact
    const double distance=number(m,"distance_mm",-5,5);
    require(std::abs(distance)>=0.5,"MoveL distance must be at least 0.5 mm");
    const double speed=number(m,"speed_mm_s",0.05,1);
    number(m,"acceleration_mm_s2",0.1,5);
    const double timeout=number(m,"motion_timeout_s",1,30);
    require(timeout>std::abs(distance)/speed+1,"MoveL timeout too short for distance/speed");
    number(m,"minimum_contact_n",0.2,10);
    number(m,"force_tolerance_n",0.1,5);
    const double target=f.at("desired_wrench").at(2);
    require(std::abs(target)>=1 && std::abs(target)<=10,"Choose signed Fz target in [1,10] N magnitude");
    require(f.at("select")==json({0,0,1,0,0,0}),"ft_movel controls Fz only");
    require(f.at("pid").at(0).get<double>()>0,"Force P must be positive");
    for(int i=0;i<6;++i) if(i!=2)
      require(f.at("desired_wrench").at(i)==0,"Only desired Fz may be nonzero");
    require(std::abs(distance)+f.at("max_dis_mm").get<double>()<g.at("max_tcp_delta_mm").get<double>(),
      "MoveL distance plus force adjustment must fit TCP guard");
    require(c.at("timeout_s").get<double>()>timeout+8,"Worker timeout must leave admission/cleanup time");
  }
}

struct Probe {
  json c, result;
  std::string name;
  std::ofstream events, samples;
  FRRobot *robot = nullptr; // constructed only after validation and --execute
  ROBOT_STATE_PKG state{}, initial{};
  int previous_frame = -1;
  Clock::time_point started = Clock::now(), last_fresh = started;
  bool connected = false, servo_started = false, torque_started = false;
  bool force_started = false, compliance_started = false, actuator_attempted = false;
  std::size_t fresh_count = 0;
  double seconds() const { return std::chrono::duration<double>(Clock::now() - started).count(); }
  int call(const std::string &api, const json &args, const std::function<int()> &fn) {
    events << json{{"event", "call_begin"}, {"t_s", seconds()}, {"api", api}, {"args", args}}.dump() << std::endl;
    const auto before = Clock::now();
    const int rc = fn();
    json e{{"event", "call_end"}, {"t_s", seconds()}, {"api", api}, {"rc", rc},
           {"elapsed_ms", std::chrono::duration<double, std::milli>(Clock::now()-before).count()}};
    result["calls"].push_back(e); events << e.dump() << std::endl;
    return rc;
  }
  void ok(int rc, const std::string &api) { require(rc == 0, api + " returned " + std::to_string(rc)); }
  void read(bool guarded = false) {
    require(!interrupted, "Interrupted");
    const int rc = robot->GetRobotRealTimeState(&state);
    require(rc == 0 && state.frame_head == 0x5A5A, "Invalid SDK state packet rc=" + std::to_string(rc));
    const auto now = Clock::now();
    const bool fresh = previous_frame != state.frame_cnt;
    if (fresh) { previous_frame = state.frame_cnt; last_fresh = now; ++fresh_count; }
    const double age = std::chrono::duration<double>(now-last_fresh).count();
    json j{{"t_s", seconds()}, {"frame", state.frame_cnt}, {"fresh", fresh}, {"age_s", age},
           {"joint_deg", state.jt_cur_pos}, {"tcp_mm_deg", state.tl_cur_pos},
           {"joint_velocity_deg_s", state.actual_qd}, {"joint_torque_nm", state.jt_cur_tor},
           {"ft_raw", state.ft_sensor_raw_data}, {"ft_reference", state.ft_sensor_data},
           {"ft_active", state.ft_sensor_active}, {"mode", state.robot_mode}, {"enabled", state.rbtEnableState},
           {"main_error", state.main_code}, {"sub_error", state.sub_code}, {"motion_done", state.motion_done}};
    samples << j.dump() << std::endl;
    for (int i = 0; i < 6; ++i) {
      require(std::isfinite(state.jt_cur_pos[i]) && std::isfinite(state.tl_cur_pos[i]) &&
              std::isfinite(state.actual_qd[i]), "Nonfinite motion feedback");
    }
    require(age <= c.at("guards").at("max_frame_age_s").get<double>(), "Stale controller state");
    if (guarded) {
      const auto &g = c.at("guards");
      require(!state.main_code && !state.sub_code && !state.EmergencyStop && !state.collisionState &&
              !state.safety_stop0_state && !state.safety_stop1_state && state.rbtEnableState,
              "Controller fault/stop/disabled during test");
      require(state.robot_mode == initial.robot_mode && state.tool == initial.tool && state.user == initial.user,
              "Mode or coordinate selection changed during test");
      double xyz = 0;
      for (int i = 0; i < 6; ++i) {
        require(std::abs(state.jt_cur_pos[i]-initial.jt_cur_pos[i]) <= g.at("max_joint_delta_deg").get<double>(), "Joint displacement guard");
        if (i < 3) xyz += std::pow(state.tl_cur_pos[i]-initial.tl_cur_pos[i], 2);
        else require(std::abs(std::remainder(state.tl_cur_pos[i]-initial.tl_cur_pos[i], 360.0)) <= g.at("max_rotation_deg").get<double>(), "Rotation guard");
        if (state.ft_sensor_active) require(std::isfinite(state.ft_sensor_raw_data[i]) &&
          std::abs(state.ft_sensor_raw_data[i]) <= g.at(i < 3 ? "max_force_n" : "max_torque_nm").get<double>(), "Force/torque observation guard");
      }
      require(std::sqrt(xyz) <= g.at("max_tcp_delta_mm").get<double>(), "TCP displacement guard");
      if (is_force(name)) require(state.ft_sensor_active == 1, "Force sensor became inactive");
    }
  }
  void observe(double duration, bool guarded = false) {
    const auto begin = Clock::now(); auto next = begin;
    while (std::chrono::duration<double>(Clock::now()-begin).count() < duration) {
      read(guarded);
      next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.at("period_s").get<double>()));
      std::this_thread::sleep_until(next);
    }
  }
  int ft_control(int flag, const std::string &variant) {
    const auto &f = c.at("force");
    auto wd = array<6>(f, "desired_wrench", -1000, 1000);
    ForceTorque wrench{wd[0],wd[1],wd[2],wd[3],wd[4],wd[5]};
    auto selected = array<6>(f, "select", 0, 1), pid_d = array<6>(f, "pid", 0, 1);
    uint8_t select[6]; float pid[6];
    for (int i = 0; i < 6; ++i) { select[i] = static_cast<uint8_t>(selected[i]); pid[i] = static_cast<float>(pid_d[i]); }
    auto m = array<2>(f, "mass_rx_ry", 0.1, 10), b = array<2>(f, "damping_rx_ry", 0.1, 50);
    auto threshold = array<2>(f, "threshold_rx_ry", 0, 10), coeff = array<2>(f, "adjust_coeff_rx_ry", 0, 1);
    return call("FT_Control/"+variant, {{"flag",flag},{"parameters",f}}, [&] {
      const int sensor=f.at("sensor_id"), filter=f.at("filter"), adapt=f.at("position_adaptation");
      const float dis=f.at("max_dis_mm"), ang=f.at("max_ang_deg");
      if (variant == "basic") return robot->FT_Control(flag,sensor,select,&wrench,pid,0,0,dis,ang,filter,adapt,1);
      if (variant == "mb") return robot->FT_Control(flag,sensor,select,&wrench,pid,0,0,dis,ang,m.data(),b.data(),0,filter,adapt,1);
      return robot->FT_Control(flag,sensor,select,&wrench,pid,0,0,dis,ang,m.data(),b.data(),threshold.data(),coeff.data(),0,filter,adapt,1);
    });
  }
  void force_movel() {
    const auto &m=c.at("force_movel"), &f=c.at("force");
    require(initial.user==0,"ft_movel requires base workpiece frame user=0");
    require(initial.tool==m.at("expected_tool").get<int>(),"Selected TCP differs from expected_tool");
    for(const auto &axis : state.extAxisStatus) require(std::isfinite(axis.pos) &&
      std::abs(axis.pos)<0.001 && std::isfinite(axis.vel) && std::abs(axis.vel)<0.001,
      "ft_movel supports unused/zero stationary external axes only");
    const double target=f.at("desired_wrench").at(2), minimum=m.at("minimum_contact_n");
    const double sign=target>0 ? 1 : -1;
    // Require sustained existing contact; this case never approaches a surface.
    const auto contact_end=Clock::now()+std::chrono::milliseconds(200);
    int contact_frames=0, frame=-1;
    while(Clock::now()<contact_end) {
      read(true);
      require(std::isfinite(state.ft_sensor_data[2]) && sign*state.ft_sensor_data[2]>=minimum,
        "No prepared contact with matching Fz sign; MoveL not started");
      if(frame!=state.frame_cnt) {frame=state.frame_cnt;++contact_frames;}
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    require(contact_frames>=3,"Insufficient fresh contact feedback");
    const double k=std::acos(-1.0)/180;
    const double rx=initial.tl_cur_pos[3]*k,ry=initial.tl_cur_pos[4]*k,rz=initial.tl_cur_pos[5]*k;
    const double cx=std::cos(rx),sx=std::sin(rx),cy=std::cos(ry),sy=std::sin(ry),cz=std::cos(rz),sz=std::sin(rz);
    const double R[3][3]{{cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx},
      {sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx},{-sy,cy*sx,cy*cx}};
    const int axis=m.at("tool_axis");
    const double distance=m.at("distance_mm");
    DescPose pose(initial.tl_cur_pos[0]+R[0][axis]*distance,
      initial.tl_cur_pos[1]+R[1][axis]*distance,initial.tl_cur_pos[2]+R[2][axis]*distance,
      initial.tl_cur_pos[3],initial.tl_cur_pos[4],initial.tl_cur_pos[5]);
    JointPos seed{},goal{};
    std::copy(initial.jt_cur_pos,initial.jt_cur_pos+6,seed.jPos);
    ok(call("GetInverseKinRef",{},[&]{return robot->GetInverseKinRef(0,&pose,&seed,&goal);}),"GetInverseKinRef");
    for(int i=0;i<6;++i) require(std::isfinite(goal.jPos[i]) &&
      std::abs(goal.jPos[i]-initial.jt_cur_pos[i])<c.at("guards").at("max_joint_delta_deg").get<double>(),
      "MoveL endpoint IK exceeds joint observation envelope");
    result["movel_target_mm_deg"]={pose.tran.x,pose.tran.y,pose.tran.z,pose.rpy.rx,pose.rpy.ry,pose.rpy.rz};
    read(true);
    require(sign*state.ft_sensor_data[2]>=minimum,"Contact lost before FT_Control");
    actuator_attempted=true;force_started=true;
    ok(ft_control(1,"basic"),"FT_Control");
    ExaxisPos ext{0,0,0,0}; DescPose offset(0,0,0,0,0,0);
    const float speed=m.at("speed_mm_s"),acc=m.at("acceleration_mm_s2");
    // blendR=0 returns without waiting; physical speed/acceleration mode=1.
    ok(call("MoveL",{{"speed_mm_s",speed},{"acceleration_mm_s2",acc},
      {"blendR",0},{"velAccParamMode",1}},[&]{return robot->MoveL(&goal,&pose,initial.tool,
        initial.user,100,100,speed,0,0,&ext,0,0,&offset,acc,1,2,10);}),"MoveL");
    const auto start=Clock::now();
    const double timeout=m.at("motion_timeout_s"), tolerance=m.at("force_tolerance_n");
    int settled=0,force_samples=0,in_band=0; frame=state.frame_cnt;
    double sum=0,squared_error=0,force_min=1e9,force_max=-1e9,progress=0;
    bool complete=false;
    auto last_contact=start;
    while(std::chrono::duration<double>(Clock::now()-start).count()<timeout) {
      read(true);
      if(frame!=state.frame_cnt) {
        frame=state.frame_cnt;progress=0;
        for(int j=0;j<3;++j) progress+=(state.tl_cur_pos[j]-initial.tl_cur_pos[j])*R[j][axis];
        require(std::isfinite(state.ft_sensor_data[2]),"Nonfinite reference Fz");
        const double force=state.ft_sensor_data[2];
        require(std::abs(force)<=c.at("guards").at("max_force_n").get<double>(),
          "Reference Fz observation guard");
        if(sign*force>=minimum) last_contact=Clock::now();
        require(std::chrono::duration<double>(Clock::now()-last_contact).count()<0.1,
          "Contact lost for 100 ms during MoveL");
        // Assess force along the middle of the measured tangential stroke.
        const double fraction=progress/distance;
        if(fraction>=0.1 && fraction<=0.9) {
          ++force_samples;sum+=force;squared_error+=std::pow(force-target,2);
          force_min=std::min(force_min,force);force_max=std::max(force_max,force);
          if(std::abs(force-target)<=tolerance) ++in_band;
        }
        const bool still=std::all_of(state.actual_qd,state.actual_qd+6,
          [](double v){return std::abs(v)<0.1;});
        settled=std::abs(progress-distance)<=0.1 && state.motion_done==1 && still ? settled+1 : 0;
        if(settled>=3) {complete=true;break;}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    result["measured_tangential_mm"]=progress;
    result["movel_complete"]=complete;
    result["force_tracking"]={{"target_fz_n",target},{"samples",force_samples},
      {"tolerance_n",tolerance},{"reference_verified_by_operator",true}};
    if(force_samples) {
      result["force_tracking"].update({{"mean_fz_n",sum/force_samples},
        {"min_fz_n",force_min},{"max_fz_n",force_max},
        {"rmse_n",std::sqrt(squared_error/force_samples)},
        {"fraction_in_tolerance",static_cast<double>(in_band)/force_samples}});
    }
    require(complete,"MoveL did not reach a settled endpoint before timeout; inspect measured motion");
    result["status"]="API_ACCEPTED_UNVERIFIED";
  }
  void cleanup() {
    if (!robot) return;
    bool good = true;
    auto clean = [&](const char *api, const std::function<int()> &fn) {
      try { if (call(api, json::object(), fn) != 0) good = false; }
      catch (...) { good = false; }
    };
    if (actuator_attempted) clean("StopMotion/cleanup", [&] { return robot->StopMotion(); });
    if (compliance_started) clean("FT_ComplianceStop", [&] { return robot->FT_ComplianceStop(); });
    if (force_started) {
      try { if (ft_control(0, name == "ft_control_basic" || name == "ft_movel" ? "basic" : name == "ft_control_mb" ? "mb" : "extended") != 0) good=false; }
      catch (...) { good=false; }
    }
    if (torque_started) clean("ServoJTEnd", [&] { return robot->ServoJTEnd(); });
    if (servo_started) clean("ServoMoveEnd", [&] { return robot->ServoMoveEnd(); });
    if (actuator_attempted) {
      const auto deadline=Clock::now()+std::chrono::seconds(2);
      int frame=-1, stationary=0;
      while(Clock::now()<deadline && stationary<3) {
        ROBOT_STATE_PKG stopped{};
        const int rc=robot->GetRobotRealTimeState(&stopped);
        if(rc==0 && stopped.frame_head==0x5A5A && stopped.frame_cnt!=frame) {
          frame=stopped.frame_cnt;
          const bool still=std::all_of(stopped.actual_qd,stopped.actual_qd+6,
            [](double v){return std::isfinite(v) && std::abs(v)<0.1;});
          stationary=still ? stationary+1 : 0;
          result["shutdown_state"]={{"frame",frame},{"joint_velocity_deg_s",stopped.actual_qd},
            {"mode",stopped.robot_mode},{"enabled",stopped.rbtEnableState}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      result["stationary_after_cleanup"]=stationary>=3;
      if(stationary<3) good=false;
    }
    if (connected) clean("CloseRPC", [&] { return robot->CloseRPC(); });
    result["cleanup_confirmed"] = good;
    // Some supplied SDKs keep receive threads alive after CloseRPC. Process exit
    // owns their lifetime; do not invoke the vendor destructor after closing.
    if (!good) result["status"] = "CLEANUP_UNCONFIRMED";
  }
  void run() {
    Dl_info info{};
    if (dladdr(dlsym(RTLD_DEFAULT, "_ZN7FRRobot3RPCEPKc"), &info) && info.dli_fname)
      result["loaded_library"] = std::filesystem::canonical(info.dli_fname).string();
#ifdef FR10_EXPECTED_SDK
    require(result.contains("loaded_library") && result.at("loaded_library")==FR10_EXPECTED_SDK,
            "Loaded FAIRINO library differs from the exact library selected at build time");
#endif
    robot = new FRRobot();
    connected = true; // even a failed RPC attempt can leave SDK sockets open
    ok(call("RPC", {{"ip",c.at("ip")}}, [&] { return robot->RPC(c.at("ip").get<std::string>().c_str()); }), "RPC");
    char sdk[256]{}, model[64]{}, web[64]{}, controller[64]{};
    ok(call("GetSDKVersion", {}, [&] { return robot->GetSDKVersion(sdk); }), "GetSDKVersion");
    ok(call("GetSoftwareVersion", {}, [&] { return robot->GetSoftwareVersion(model,web,controller); }), "GetSoftwareVersion");
    result["versions"] = {{"sdk",sdk},{"model",model},{"web",web},{"controller",controller}};
    const auto before = Clock::now();
    while (true) {
      const int rc = robot->GetRobotRealTimeState(&state);
      if (rc == 0 && state.frame_head == 0x5A5A) break;
      require(!interrupted && std::chrono::duration<double>(Clock::now()-before).count() < 3, "No valid initial state within 3 seconds");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    initial = state; previous_frame=-1; last_fresh=Clock::now(); read();
    result["initial"] = {{"joint_deg",initial.jt_cur_pos},{"tcp_mm_deg",initial.tl_cur_pos},
      {"mode",initial.robot_mode},{"enabled",initial.rbtEnableState},{"tool",initial.tool},{"user",initial.user},
      {"ft_active",initial.ft_sensor_active},{"payload_kg",initial.load},{"payload_cog_mm",initial.loadCog},
      {"tool_coordinate",initial.toolCoord},{"workpiece_coordinate",initial.wobjCoord}};
    json mismatches=json::array();
    if (std::string(web) != c.at("expected_web_version").get<std::string>()) mismatches.push_back("web version");
    if (std::string(sdk).find(c.at("expected_sdk_contains").get<std::string>()) == std::string::npos) mismatches.push_back("SDK version");
    if (std::string(model).rfind(c.at("expected_model_prefix").get<std::string>(),0) != 0) mismatches.push_back("robot model");
    result["identity_mismatches"]=mismatches;
    const bool mutating = name != "state" && name != "ft_read";
    if (mutating) {
      require(mismatches.empty(), "Identity mismatch; inspect versions before testing");
      require(!state.main_code && !state.sub_code && !state.EmergencyStop && !state.collisionState &&
        !state.safety_stop0_state && !state.safety_stop1_state, "Controller has fault/stop/collision");
      require(state.program_state == 1, "Stop the cabinet program before the test");
      const auto admission_end=Clock::now()+std::chrono::milliseconds(100);
      while(Clock::now()<admission_end) {
        read();
        for (double v : state.actual_qd) require(std::abs(v) < 0.1, "Robot must be stationary at admission");
        require(!state.main_code && !state.sub_code && !state.EmergencyStop && !state.collisionState &&
          !state.safety_stop0_state && !state.safety_stop1_state,"Fault during admission");
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
      }
      if (is_motion(name)) {
        require(state.rbtEnableState == 1, "Enable must be selected by operator");
        if (name.rfind("servojt",0) == 0) {
          uint8_t drag=0;
          ok(call("IsInDragTeach",{},[&] { return robot->IsInDragTeach(&drag); }), "IsInDragTeach");
          require(drag == 1, "Prepare torque/drag mode per vendor procedure; bench never switches it");
        } else {
          require(state.robot_mode == 0, "Auto mode must be selected by operator");
          require(state.robot_state == 1, "Cabinet must report stopped motion before admission");
        }
      }
      if(name=="servoj" || name.rfind("servocart_",0)==0)
        for(const auto &axis : state.extAxisStatus) require(std::isfinite(axis.pos) &&
          std::abs(axis.pos)<0.001 && std::isfinite(axis.vel) && std::abs(axis.vel)<0.001,
          "This bench supports unused/zero stationary external axes only");
      if (is_force(name) || name.rfind("ft_",0) == 0) require(state.ft_sensor_active == 1, "Force sensor inactive");
      read(is_motion(name));
    }
    const double duration=c.at("duration_s"), period=c.at("period_s");
    if (name == "state" || name == "ft_read") {
      if (name == "ft_read") {
        int company=0,device=0,version=0,bus=0;
        ok(call("FT_GetConfig",{},[&] {return robot->FT_GetConfig(&company,&device,&version,&bus);}),"FT_GetConfig");
        result["ft_config"]={company,device,version,bus};
        ForceTorque raw{}, reference{};
        ok(call("FT_GetForceTorqueOrigin",{{"flag",1}},[&] {return robot->FT_GetForceTorqueOrigin(1,&raw);}),"FT_GetForceTorqueOrigin");
        ok(call("FT_GetForceTorqueRCS",{{"flag",1}},[&] {return robot->FT_GetForceTorqueRCS(1,&reference);}),"FT_GetForceTorqueRCS");
        result["ft_getter_raw"]={raw.fx,raw.fy,raw.fz,raw.tx,raw.ty,raw.tz};
        result["ft_getter_reference"]={reference.fx,reference.fy,reference.fz,reference.tx,reference.ty,reference.tz};
        for (const auto &values : {result["ft_getter_raw"],result["ft_getter_reference"]})
          for (const auto &value : values) require(value.is_number() && std::isfinite(value.get<double>()),"Nonfinite force getter result");
      }
      observe(duration);
      result["status"]=name == "ft_read" && !state.ft_sensor_active ? "SENSOR_INACTIVE" : "OBSERVED";
      if(!mismatches.empty()) result["status"]="IDENTITY_MISMATCH";
    } else if (name == "servoj" || name.rfind("servocart_",0) == 0) {
      const auto &v=c.at("servo");
      servo_started=true; actuator_attempted=true;
      ok(call("ServoMoveStart",{},[&] {return robot->ServoMoveStart();}),"ServoMoveStart");
      ExaxisPos ext{0,0,0,0};
      const int axis=v.at(name=="servoj" ? "joint_index" : "cart_axis");
      const double delta=v.at(name=="servoj" ? "joint_delta_deg" : "cart_delta_mm");
      const int count=std::max(2,static_cast<int>(std::ceil(duration/period)));
      const int hold_count=static_cast<int>(std::ceil(0.1/period));
      auto next=Clock::now();
      const double start=seconds();
      for (int i=0;i<count+hold_count;++i) {
        read(true);
        const double u=std::min(1.0,static_cast<double>(i+1)/count);
        const double previous=std::min(1.0,static_cast<double>(i)/count);
        const auto smooth=[](double x) {return x*x*x*(10+x*(-15+6*x));};
        if (name=="servoj") {
          JointPos joint;
          std::copy(initial.jt_cur_pos,initial.jt_cur_pos+6,joint.jPos);
          joint.jPos[axis]+=delta*smooth(u);
          ok(call("ServoJ",{{"joint_deg",joint.jPos},{"exaxis",{0,0,0,0}},{"cmdT",period},{"id",0}},[&] {
            return robot->ServoJ(&joint,&ext,0,0,static_cast<float>(period),0,0,0);}),"ServoJ");
        } else {
          const int mode=name=="servocart_absolute" ? 0 : name=="servocart_base" ? 1 : 2;
          double xyz[3]{0,0,0}; Rpy rpy;
          if (mode==0) {std::copy(initial.tl_cur_pos,initial.tl_cur_pos+3,xyz); rpy={initial.tl_cur_pos[3],initial.tl_cur_pos[4],initial.tl_cur_pos[5]};}
          xyz[axis]+=delta*(mode==0 ? smooth(u) : smooth(u)-smooth(previous));
          DescPose pose(xyz[0],xyz[1],xyz[2],rpy.rx,rpy.ry,rpy.rz);
          float gain[6]{1,1,1,1,1,1};
          ok(call("ServoCart",{{"mode",mode},{"pose_mm_deg",{xyz[0],xyz[1],xyz[2],rpy.rx,rpy.ry,rpy.rz}},
            {"exaxis",{0,0,0,0}},{"pos_gain",{1,1,1,1,1,1}},{"cmdT",period}},[&] {
            return robot->ServoCart(mode,&pose,ext,gain,0,0,static_cast<float>(period),0,0);}),"ServoCart");
        }
        next+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period));
        require(Clock::now() <= next+std::chrono::duration<double>(0.05), "Stream missed deadline by >50 ms; inspect call latency");
        std::this_thread::sleep_until(next);
      }
      result["stream_elapsed_s"]=seconds()-start; result["stream_commands"]=count+hold_count;
      ok(call("ServoMoveEnd",{},[&] {return robot->ServoMoveEnd();}),"ServoMoveEnd");
      servo_started=false;
      observe(0.1,true);
      double measured=state.jt_cur_pos[axis]-initial.jt_cur_pos[axis];
      if (name!="servoj") {
        double direction[3]{0,0,0}; direction[axis]=1;
        if (name=="servocart_tool") {
          const double k=std::acos(-1.0)/180;
          const double rx=initial.tl_cur_pos[3]*k,ry=initial.tl_cur_pos[4]*k,rz=initial.tl_cur_pos[5]*k;
          const double cx=std::cos(rx),sx=std::sin(rx),cy=std::cos(ry),sy=std::sin(ry),cz=std::cos(rz),sz=std::sin(rz);
          const double R[3][3]{{cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx},
            {sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx},{-sy,cy*sx,cy*cx}};
          for(int j=0;j<3;++j) direction[j]=R[j][axis];
        }
        measured=0; double residual2=0;
        for(int j=0;j<3;++j) {const double d=state.tl_cur_pos[j]-initial.tl_cur_pos[j]; measured+=d*direction[j]; residual2+=std::pow(d-delta*direction[j],2);}
        result["final_target_error_mm"]=std::sqrt(residual2);
      }
      result["requested_delta"]=delta; result["measured_delta"]=measured;
      const double minimum=v.at(name=="servoj" ? "minimum_joint_response_deg" : "minimum_cart_response_mm");
      const double tolerance=v.at(name=="servoj" ? "final_joint_tolerance_deg" : "final_cart_tolerance_mm");
      const double error=name=="servoj" ? std::abs(measured-delta) : result.at("final_target_error_mm").get<double>();
      result["status"]=std::abs(measured)<minimum ? "NO_MEASURED_RESPONSE" :
        measured*delta<=0 || error>tolerance ? "TRACKING_MISMATCH" : "MOTION_OBSERVED";
    } else if (name.rfind("servojt",0)==0) {
      const auto &t=c.at("torque"); auto td=array<6>(t,"torque_nm",-500,500);
      float torque[6]; for(int i=0;i<6;++i) torque[i]=static_cast<float>(td[i]);
      auto power=array<6>(t,"power_limit_w",0.001,5000), vel=array<6>(t,"velocity_limit_deg_s",0.001,10);
      torque_started=true; actuator_attempted=true;
      ok(call("ServoJTStart",{},[&] {return robot->ServoJTStart();}),"ServoJTStart");
      auto next=Clock::now();
      const int count=std::max(2,static_cast<int>(std::ceil(duration/period)));
      for(int i=0;i<count;++i) {
        read(true);
        for(int j=0;j<6;++j) require(std::abs(state.actual_qd[j])<=vel[j],"Torque velocity guard");
        ok(call(name=="servojt" ? "ServoJT/basic" : "ServoJT/limits",{{"torque_nm",td},{"interval",period},
          {"checkFlag",name=="servojt" ? 0 : 3},{"power_limit_w",power},{"velocity_limit_deg_s",vel}},[&] {
          return name=="servojt" ? robot->ServoJT(torque,period) : robot->ServoJT(torque,period,3,power.data(),vel.data());}),"ServoJT");
        next+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period));
        require(Clock::now() <= next+std::chrono::duration<double>(0.05),"Torque stream missed deadline");
        std::this_thread::sleep_until(next);
      }
      result["status"]="API_ACCEPTED_UNVERIFIED";
    } else if (name=="ft_zero" || name=="ft_payload_record" || name=="ft_payload_compute") {
      const auto &k=c.at("calibration");
      if(name=="ft_zero") ok(call("FT_SetZero",{{"action",k.at("zero_action")}},[&] {return robot->FT_SetZero(k.at("zero_action").get<uint8_t>());}),"FT_SetZero");
      else if(name=="ft_payload_record") ok(call("FT_PdIdenRecord",{{"sensor_coordinate_id",k.at("sensor_coordinate_id")}},[&] {return robot->FT_PdIdenRecord(k.at("sensor_coordinate_id"));}),"FT_PdIdenRecord");
      else {float weight=0;ok(call("FT_PdIdenCompute",{},[&] {return robot->FT_PdIdenCompute(&weight);}),"FT_PdIdenCompute");
        require(std::isfinite(weight) && weight>=0,"Invalid payload result");result["payload_weight_kg"]=weight;}
      observe(0.1); result["status"]="API_ACCEPTED_UNVERIFIED";
    } else if (name=="ft_movel") {
      force_movel();
    } else {
      actuator_attempted=true;
      const auto &f=c.at("force");
      if(name=="ft_compliance") {
        force_started=true;
        ok(ft_control(1,"extended"),"FT_Control");
        compliance_started=true;
        ok(call("FT_ComplianceStart",{{"p",f.at("compliance_p")},{"threshold_n",f.at("compliance_threshold_n")}},[&] {
          return robot->FT_ComplianceStart(f.at("compliance_p"),f.at("compliance_threshold_n"));}),"FT_ComplianceStart");
        observe(duration,true);
      } else {
        force_started=true;
        ok(ft_control(1,name=="ft_control_basic" ? "basic" : name=="ft_control_mb" ? "mb" : "extended"),"FT_Control");
        read(true);
        if(name=="ft_spiral") ok(call("FT_SpiralSearch",f,[&] {return robot->FT_SpiralSearch(f.at("rcs"),f.at("spiral_dr_mm"),f.at("threshold_n"),f.at("spiral_max_time_ms"),f.at("speed_mm_s"));}),"FT_SpiralSearch");
        else if(name=="ft_linear") ok(call("FT_LinInsertion",f,[&] {return robot->FT_LinInsertion(f.at("rcs"),f.at("threshold_n"),f.at("speed_mm_s"),0,f.at("max_distance_mm"),f.at("linear_direction").get<uint8_t>());}),"FT_LinInsertion");
        else if(name=="ft_rotational") ok(call("FT_RotInsertion",f,[&] {return robot->FT_RotInsertion(f.at("rcs"),f.at("rotation_speed_deg_s"),f.at("threshold_n"),f.at("max_angle_deg"),f.at("rotation_force_direction").get<uint8_t>(),0,f.at("rotation_direction").get<uint8_t>(),0);}),"FT_RotInsertion");
        else if(name=="ft_surface") ok(call("FT_FindSurface",f,[&] {return robot->FT_FindSurface(f.at("rcs"),f.at("surface_direction").get<uint8_t>(),f.at("surface_axis").get<uint8_t>(),f.at("speed_mm_s"),0,f.at("max_distance_mm"),f.at("threshold_n"));}),"FT_FindSurface");
        observe(duration,true);
      }
      result["status"]="API_ACCEPTED_UNVERIFIED";
    }
    result["final"]={{"joint_deg",state.jt_cur_pos},{"tcp_mm_deg",state.tl_cur_pos},
      {"ft_raw",state.ft_sensor_raw_data},{"ft_reference",state.ft_sensor_data}};
    result["fresh_state_frames"]=fresh_count;
    require(fresh_count>=2,"Insufficient fresh feedback to assess result");
  }
};

int main(int argc,char **argv) {
  std::string config, name, output;
  bool execute=false,exclusive=false,validate_only=false;
  try {
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i];
      if(a=="--execute") execute=true;
      else if(a=="--validate-only") validate_only=true;
      else if(a=="--exclusive-control") exclusive=true;
      else if((a=="--config" || a=="--case" || a=="--output") && i+1<argc) {
        const std::string v=argv[++i];
        if(a=="--config") config=v; else if(a=="--case") name=v; else output=v;
      } else throw std::runtime_error("Unknown/incomplete option: "+a);
    }
    require(!config.empty() && !name.empty() && !output.empty(),"Required: --config FILE --case NAME --output DIR");
    std::ifstream in(config); require(in.good(),"Cannot read config"); json c; in>>c;
    validate(c,name);
    if(validate_only) {std::cout<<"Configuration valid; no SDK object constructed\n";return 0;}
    require(execute,"No connection opened. Add --execute to contact the configured endpoint");
    require(name=="state" || name=="ft_read" || exclusive,"Mutating case requires --exclusive-control (stop ROS/other command clients first)");
    std::filesystem::create_directories(output);
    Probe p; p.c=c;p.name=name;
    p.result={{"case",name},{"target_label",c.at("target")},{"configuration",c},{"calls",json::array()},
      {"status","FAILED"},{"cleanup_confirmed",false},{"physical_behavior_certified",false}};
#ifdef FR10_OFFLINE_FAKE
    p.result["backend"]="OFFLINE_FAKE";
#else
    p.result["backend"]="FAIRINO_SDK";
#endif
    p.events.open(output+"/events.jsonl");p.samples.open(output+"/samples.jsonl");
    require(p.events.good() && p.samples.good(),"Cannot open result files");
    std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
    try {p.run();} catch(const std::exception &e) {p.result["error"]=e.what();p.result["status"]="FAILED";}
    p.cleanup();
    p.result["elapsed_s"]=p.seconds();
    std::ofstream out(output+"/result.json");out<<p.result.dump(2)<<std::endl;out.close();
    std::cout<<p.result.dump()<<std::endl;
    const std::string status=p.result.at("status");
    const int code=status=="OBSERVED" || status=="MOTION_OBSERVED" || status=="API_ACCEPTED_UNVERIFIED" ? 0 : 1;
    p.events.close();p.samples.close();std::cout.flush();std::cerr.flush();
    std::_Exit(code); // vendor SDK thread lifetime is bounded by this worker process
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
