// Offline fault-injection double. Never opens a socket or loads libfairino.
#include "robot.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

static ROBOT_STATE_PKG s{};
static bool moving=false;
static int move_tick=0;
static double move_start[3]{},move_goal[3]{};
static bool present(const char *key) { return std::getenv(key) != nullptr; }
static int record(const std::string &name) {
  if (const char *file=std::getenv("BENCH_FAKE_LOG")) std::ofstream(file,std::ios::app)<<name<<'\n';
  const char *fail=std::getenv("BENCH_FAKE_FAIL");
  return fail && name==fail ? 3 : 0;
}
FRRobot::FRRobot() {
  record("constructor");s={};s.frame_head=0x5A5A;s.program_state=1;s.robot_state=present("BENCH_FAKE_DRAG") ? 4 : 1;
  moving=false;move_tick=0;
  s.motion_done=1;
  if(present("BENCH_FAKE_CONTACT")) s.ft_sensor_data[2]=2;
  s.robot_mode=present("BENCH_FAKE_DRAG") ? 1 : 0;s.rbtEnableState=1;
  s.ft_sensor_active=present("BENCH_FAKE_SENSOR_OFF") ? 0 : 1;
  const double q[]{0,-90,90,-90,-90,0},p[]{100,200,300,0,0,90};
  for(int i=0;i<6;++i) {s.jt_cur_pos[i]=q[i];s.tl_cur_pos[i]=p[i];}
}
FRRobot::~FRRobot() = default;
errno_t FRRobot::RPC(const char *) {return record("RPC");}
errno_t FRRobot::CloseRPC() {return record("CloseRPC");}
errno_t FRRobot::GetSDKVersion(char *out) {std::strcpy(out,"SDK V2.3.3.0-robot v3.9.3");return record("GetSDKVersion");}
errno_t FRRobot::GetSoftwareVersion(char model[64],char web[64],char controller[64]) {
  std::strcpy(model,"FR10-TEST");std::strcpy(web,present("BENCH_FAKE_WRONG_VERSION") ? "v3.9.8" : "v3.9.3");
  std::strcpy(controller,"FAKE-NOT-HARDWARE");return record("GetSoftwareVersion");
}
errno_t FRRobot::GetRobotRealTimeState(ROBOT_STATE_PKG *out) {
  if(!present("BENCH_FAKE_STALE")) ++s.frame_cnt;
  if(moving) {
    ++move_tick;
    const double u=present("BENCH_FAKE_NO_RESPONSE") ? 0 : std::min(1.0,move_tick/20.0);
    for(int i=0;i<3;++i) s.tl_cur_pos[i]=move_start[i]+u*(move_goal[i]-move_start[i]);
    if(present("BENCH_FAKE_FORCE_SPIKE")) s.ft_sensor_raw_data[2]=100;
    if(present("BENCH_FAKE_SENSOR_LOSS")) s.ft_sensor_active=0;
    if(present("BENCH_FAKE_CONTACT_LOSS")) s.ft_sensor_data[2]=0;
    if(u>=1) {moving=false;s.motion_done=1;}
  }
  *out=s;return 0;
}
errno_t FRRobot::IsInDragTeach(uint8_t *out) {*out=present("BENCH_FAKE_DRAG") ? 1 : 0;return record("IsInDragTeach");}
errno_t FRRobot::StopMotion() {moving=false;s.motion_done=1;return record("StopMotion");}
errno_t FRRobot::GetInverseKinRef(int,DescPose*,JointPos *seed,JointPos *goal) {
  *goal=*seed;return record("GetInverseKinRef");
}
errno_t FRRobot::MoveL(JointPos*,DescPose *pose,int,int,float,float,float,float,int,
    ExaxisPos*,uint8_t,uint8_t,DescPose*,float,int,int,int) {
  const int rc=record("MoveL");
  if(rc) return rc;
  for(int i=0;i<3;++i) move_start[i]=s.tl_cur_pos[i];
  move_goal[0]=pose->tran.x;move_goal[1]=pose->tran.y;move_goal[2]=pose->tran.z;
  moving=true;move_tick=0;s.motion_done=0;
  return 0;
}
errno_t FRRobot::ServoMoveStart() {return record("ServoMoveStart");}
errno_t FRRobot::ServoMoveEnd() {return record("ServoMoveEnd");}
errno_t FRRobot::ServoJ(JointPos *q,ExaxisPos *,float,float,float,float,float,int) {
  const int rc=record("ServoJ");
  if(!rc && !present("BENCH_FAKE_NO_RESPONSE")) for(int i=0;i<6;++i) s.jt_cur_pos[i]=q->jPos[i];
  return rc;
}
errno_t FRRobot::ServoCart(int mode,DescPose *p,ExaxisPos,float gain[6],float,float,float,float,float) {
  const int rc=record("ServoCart");if(rc || present("BENCH_FAKE_NO_RESPONSE")) return rc;
  if(mode==0) {
    const double v[]{p->tran.x,p->tran.y,p->tran.z,p->rpy.rx,p->rpy.ry,p->rpy.rz};
    for(int i=0;i<6;++i) s.tl_cur_pos[i]=v[i];
  } else if(mode==1) {s.tl_cur_pos[0]+=gain[0]*p->tran.x;s.tl_cur_pos[1]+=gain[1]*p->tran.y;s.tl_cur_pos[2]+=gain[2]*p->tran.z;}
  else { // Test fixture TCP has Rz=90 deg: tool X -> base Y.
    s.tl_cur_pos[0]-=gain[1]*p->tran.y;s.tl_cur_pos[1]+=gain[0]*p->tran.x;s.tl_cur_pos[2]+=gain[2]*p->tran.z;
  }
  return rc;
}
errno_t FRRobot::ServoJTStart() {return record("ServoJTStart");}
errno_t FRRobot::ServoJTEnd() {return record("ServoJTEnd");}
errno_t FRRobot::ServoJT(float[],double) {return record("ServoJT/basic");}
errno_t FRRobot::ServoJT(float[],double,int,double[6],double[6]) {return record("ServoJT/limits");}
errno_t FRRobot::FT_GetConfig(int *a,int *b,int *c,int *d) {*a=20;*b=0;*c=0;*d=0;return record("FT_GetConfig");}
errno_t FRRobot::FT_GetForceTorqueOrigin(uint8_t,ForceTorque *out) {*out={};return record("FT_GetForceTorqueOrigin");}
errno_t FRRobot::FT_GetForceTorqueRCS(uint8_t,ForceTorque *out) {*out={};return record("FT_GetForceTorqueRCS");}
errno_t FRRobot::FT_SetZero(uint8_t) {return record("FT_SetZero");}
errno_t FRRobot::FT_PdIdenRecord(int) {return record("FT_PdIdenRecord");}
errno_t FRRobot::FT_PdIdenCompute(float *out) {*out=0.5F;return record("FT_PdIdenCompute");}
errno_t FRRobot::FT_Control(uint8_t flag,int,uint8_t[6],ForceTorque*,float[6],uint8_t,uint8_t,float,float,int,int,int) {return record(flag ? "FT_Control/basic/start" : "FT_Control/basic/stop");}
errno_t FRRobot::FT_Control(uint8_t flag,int,uint8_t[6],ForceTorque*,float[6],uint8_t,uint8_t,float,float,double[2],double[2],double,int,int,int) {return record(flag ? "FT_Control/mb/start" : "FT_Control/mb/stop");}
errno_t FRRobot::FT_Control(uint8_t flag,int,uint8_t[6],ForceTorque*,float[6],uint8_t,uint8_t,float,float,double[2],double[2],double[2],double[2],double,int,int,int) {return record(flag ? "FT_Control/extended/start" : "FT_Control/extended/stop");}
errno_t FRRobot::FT_ComplianceStart(float,float) {return record("FT_ComplianceStart");}
errno_t FRRobot::FT_ComplianceStop() {return record("FT_ComplianceStop");}
errno_t FRRobot::FT_SpiralSearch(int,float,float,float,float) {return record("FT_SpiralSearch");}
errno_t FRRobot::FT_LinInsertion(int,float,float,float,float,uint8_t) {return record("FT_LinInsertion");}
errno_t FRRobot::FT_RotInsertion(int,float,float,float,uint8_t,float,uint8_t,int) {return record("FT_RotInsertion");}
errno_t FRRobot::FT_FindSurface(int,uint8_t,uint8_t,float,float,float,float) {return record("FT_FindSurface");}
