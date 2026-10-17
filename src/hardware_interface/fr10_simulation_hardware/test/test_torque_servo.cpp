#include <limits>
#include "fr10_simulation_hardware/torque_servo.hpp"
int main() {
  using fr10_simulation_hardware::TorqueServo;
  TorqueServo servo(1000,500,20,150,150);
  double previous=servo.update(0,.005,.03,.001);
  double flipped=servo.update(0,-.005,.03,.001);
  if (std::abs(previous)>10 || std::abs(flipped)>10) return 1;
  if (servo.update(1,0,0,.001)!=150) return 2;
  try {servo.update(std::numeric_limits<double>::quiet_NaN(),0,0,.001);return 3;}
  catch (const std::invalid_argument &) {}
  TorqueServo gravity(1000,500,100,150,150);
  double q=0,v=0;
  for (int i=0;i<30000;++i) {
    const double torque=gravity.update(.1,q,v,.001);
    v+=(torque-60)*.001/5.;q+=v*.001;
    if (!std::isfinite(q) || std::abs(q)>1.) return 4;
  }
  if (std::abs(q-.1)>.0001 || std::abs(v)>.0001) return 5;
  using fr10_simulation_hardware::gravityCompensationMoment;
  if(std::abs(gravityCompensationMoment({0,1,0},{1,2,3},{1.5,2,3},3,{0,0,-10})+15)>1e-12 ||
     std::abs(gravityCompensationMoment({1,0,0},{1,2,3},{1,2.5,3},3,{0,0,-10})-15)>1e-12 ||
     gravityCompensationMoment({0,0,1},{1,2,3},{1.5,2,3},3,{0,0,-10})!=0) return 6;
  TorqueServo compensated(1000,0,100,0,150);
  q=.1;v=0.;
  for(int i=0;i<10000;++i) {
    const double torque=compensated.update(.1,q,v,.001,60);
    v+=(torque-60)*.001/5.;q+=v*.001;
  }
  if(q!=.1 || v!=0) return 7;
  // An external seating load has a bounded proportional deflection, without
  // storing an integral that persists after that load is released.
  for(int i=0;i<10000;++i) {
    v+=(compensated.update(.1,q,v,.001,60)-75)*.001/5.;q+=v*.001;
  }
  if(std::abs(q-.085)>1e-6) return 8;
  for(int i=0;i<10000;++i) {
    v+=(compensated.update(.05,q,v,.001,60)-60)*.001/5.;q+=v*.001;
  }
  if(std::abs(q-.05)>1e-6 || std::abs(v)>1e-6) return 9;
  if(compensated.update(1,0,0,.001,100)!=150) return 10;
  try {compensated.update(0,0,0,.001,std::numeric_limits<double>::quiet_NaN());return 11;}
  catch(const std::invalid_argument &) {}
  try {gravityCompensationMoment({0,0,0},{0,0,0},{1,0,0},3,{0,0,-10});return 12;}
  catch(const std::invalid_argument &) {}
  using namespace fr10_simulation_hardware;
  JointVector kp,kd,limits;
  kp<<35000,45000,35000,9000,7000,5000;
  kd<<390,495,390,99,85,71;
  limits<<150,150,150,28,28,28;
  StableTorqueServo stable(kp,kd,limits);
  // A coupled, low-inertia wrist must converge without the alternating
  // saturated torques produced by explicit damping at these same gains.
  JointInertia mass=JointInertia::Identity();
  mass.diagonal()<<10,15,3,.04,.02,.01;
  mass(3,4)=mass(4,3)=.01;mass(4,5)=mass(5,4)=.005;
  JointVector position=JointVector::Zero(),velocity=JointVector::Zero();
  JointVector target=JointVector::Constant(.01),weight=JointVector::Zero();
  weight[1]=60.;
  const Eigen::LLT<JointInertia> inverse(mass);
  for(int i=0;i<10000;++i) {
    const auto torque=stable.update(target,position,velocity,.001,mass,weight);
    if((torque.cwiseAbs().array()>limits.array()).any()) return 13;
    velocity+=.001*inverse.solve(torque-weight);position+=.001*velocity;
    if(!position.allFinite() || position.cwiseAbs().maxCoeff()>.02) return 14;
  }
  if((position-target).norm()>1e-8 || velocity.norm()>1e-8) return 15;
  // After a physical external load is released, no stored contact integral
  // remains; the same controller returns to its unloaded datum.
  JointVector contact=JointVector::Zero();contact[5]=1.;
  for(int i=0;i<10000;++i) {
    velocity+=.001*inverse.solve(stable.update(target,position,velocity,.001,mass,weight)-weight-contact);
    position+=.001*velocity;
  }
  if((position-target).norm()>.003) return 16;
  for(int i=0;i<10000;++i) {
    velocity+=.001*inverse.solve(stable.update(target,position,velocity,.001,mass,weight)-weight);
    position+=.001*velocity;
  }
  if((position-target).norm()>1e-8 || velocity.norm()>1e-8) return 17;
  try {stable.update(target,position,velocity,.001,JointInertia::Zero(),weight);return 18;}
  catch(const std::invalid_argument &) {}
  try {stable.update(target,position,velocity,.02,mass,weight);return 19;}
  catch(const std::invalid_argument &) {}
  JointEncoderVelocity encoder;
  JointVector measured=JointVector::Constant(1.);
  encoder.initialize(measured,0.);
  if(!encoder.observe(measured,.008).isZero()) return 20;
  JointVector moving;moving<<.01,-.02,.03,-.04,.05,-.06;
  measured+=.008*moving;
  if((encoder.observe(measured,.016)-moving).norm()>1e-12) return 21;
  // Identical source samples cannot fabricate a zero velocity or renew time.
  if((encoder.observe(measured,.016)-moving).norm()>1e-12) return 22;
  try {encoder.observe(measured+moving,.016);return 23;}
  catch(const std::invalid_argument &) {}
  try {encoder.observe(measured,.015);return 24;}
  catch(const std::invalid_argument &) {}
  try {encoder.observe(measured,.05);return 25;}
  catch(const std::invalid_argument &) {}
  JointVector invalid=measured;invalid[2]=std::numeric_limits<double>::quiet_NaN();
  try {encoder.observe(invalid,.024);return 26;}
  catch(const std::invalid_argument &) {}
  // Invalid samples leave the last actual encoder/source boundary intact.
  measured+=.008*moving;
  if((encoder.observe(measured,.024)-moving).norm()>1e-12) return 27;
  if(!encoder.observe(measured,.032).isZero()) return 28;
  // The 1 ms plant servo uses encoder displacement too. A constant pose has
  // zero measured velocity even if ODE's constraint velocity has a bias;
  // a real moving plant still converges with the same coupled SPD law.
  JointEncoderVelocity plant_encoder;
  position.setZero();velocity.setZero();
  plant_encoder.initialize(position,0.);
  for(int i=0;i<10000;++i) {
    const auto measured_velocity=plant_encoder.observe(position,i*.001);
    const auto torque=stable.update(target,position,measured_velocity,.001,mass,weight);
    velocity+=.001*inverse.solve(torque-weight);position+=.001*velocity;
    if(!position.allFinite() || position.cwiseAbs().maxCoeff()>.02) return 29;
  }
  if((position-target).norm()>1e-8 || velocity.norm()>1e-8) return 30;
  return 0;
}
