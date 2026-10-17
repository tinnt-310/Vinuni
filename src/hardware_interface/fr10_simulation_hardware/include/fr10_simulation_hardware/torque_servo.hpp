#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <control_toolbox/pid.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

namespace fr10_simulation_hardware {
using JointVector = Eigen::Matrix<double,6,1>;
using JointInertia = Eigen::Matrix<double,6,6>;

// ODE constraint correction can change Position() independently of its raw
// generalized velocity. Expose the actual encoder displacement per source
// interval to ros2_control and to the physics-step encoder servo.
class JointEncoderVelocity {
 public:
  void initialize(const JointVector &position,double source) {
    if(!position.allFinite() || !std::isfinite(source) || source<0.)
      throw std::invalid_argument("invalid encoder initialization");
    previous_=position;source_=source;velocity_.setZero();initialized_=true;
  }
  JointVector observe(const JointVector &position,double source) {
    if(!initialized_ || !position.allFinite() || !std::isfinite(source))
      throw std::invalid_argument("invalid encoder feedback");
    const double dt=source-source_;
    if(dt==0. && (position.array()==previous_.array()).all()) return velocity_;
    if(dt<=0. || dt>.02)
      throw std::invalid_argument("invalid encoder source interval");
    JointVector velocity=(position-previous_)/dt;
    if(!velocity.allFinite()) throw std::invalid_argument("nonfinite encoder velocity");
    previous_=position;source_=source;velocity_=velocity;
    return velocity;
  }
 private:
  JointVector previous_=JointVector::Zero(),velocity_=JointVector::Zero();
  double source_{0.};
  bool initialized_{false};
};

// Implicit damping for a torque-actuated articulated plant. M is assembled
// from measured physical link poses/inertias, including off-diagonal coupling.
// Based on Tan/Liu/Turk, Stable PD Controllers (2011), equations 10-11.
// Gravity is canceled separately; unmodeled contact remains a physical load.
class StableTorqueServo {
 public:
  StableTorqueServo(const JointVector &kp,const JointVector &kd,const JointVector &limits)
      : kp_(kp),kd_(kd),limits_(limits) {
    if(!kp.allFinite() || !kd.allFinite() || !limits.allFinite() ||
       (kp.array()<=0.).any() || (kd.array()<0.).any() || (limits.array()<=0.).any())
      throw std::invalid_argument("invalid stable servo calibration");
  }
  JointVector update(const JointVector &target,const JointVector &position,
      const JointVector &velocity,double dt,const JointInertia &mass,const JointVector &gravity) const {
    if(!target.allFinite() || !position.allFinite() || !velocity.allFinite() ||
       !gravity.allFinite() || !mass.allFinite() || !std::isfinite(dt) || dt<=0. || dt>.02 ||
       !mass.isApprox(mass.transpose(),1e-10) || (kd_.array()<kp_.array()*dt).any())
      throw std::invalid_argument("invalid stable servo feedback/inertia/period");
    Eigen::LLT<JointInertia> physical(mass);
    if(physical.info()!=Eigen::Success) throw std::invalid_argument("joint inertia is not positive definite");
    JointInertia implicit=mass;
    implicit.diagonal()+=dt*kd_;
    Eigen::LLT<JointInertia> solve(implicit);
    const JointVector drive=(kp_.array()*(target-position-dt*velocity).array()-kd_.array()*velocity.array()).matrix();
    const JointVector acceleration=solve.solve(drive);
    JointVector torque=gravity+drive-dt*(kd_.array()*acceleration.array()).matrix();
    if(solve.info()!=Eigen::Success || !torque.allFinite()) throw std::runtime_error("stable servo solve failed");
    for(unsigned i=0;i<6;++i) torque[i]=std::clamp(torque[i],-limits_[i],limits_[i]);
    return torque;
  }
 private:
  JointVector kp_,kd_,limits_;
};
inline double gravityCompensationMoment(const std::array<double,3> &axis,
    const std::array<double,3> &anchor,const std::array<double,3> &center,
    double mass,const std::array<double,3> &gravity) {
  if(!std::isfinite(mass) || mass<0.) throw std::invalid_argument("invalid link mass");
  std::array<double,3> r{},force{};
  double axis_norm=0.;
  for(unsigned i=0;i<3;++i) {
    if(!std::isfinite(axis[i]) || !std::isfinite(anchor[i]) ||
       !std::isfinite(center[i]) || !std::isfinite(gravity[i]))
      throw std::invalid_argument("invalid gravity geometry");
    axis_norm+=axis[i]*axis[i];r[i]=center[i]-anchor[i];force[i]=mass*gravity[i];
  }
  if(std::abs(axis_norm-1.)>1e-6) throw std::invalid_argument("joint axis must be normalized");
  return -(axis[0]*(r[1]*force[2]-r[2]*force[1])+
           axis[1]*(r[2]*force[0]-r[0]*force[2])+
           axis[2]*(r[0]*force[1]-r[1]*force[0]));
}
class TorqueServo {
 public:
  TorqueServo(double kp,double ki,double kd,double integral_limit,double effort_limit)
      : limit_(effort_limit) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        !std::isfinite(integral_limit) || !std::isfinite(limit_) || kp<=0 || ki<0 ||
        kd<0 || integral_limit<0 || limit_<=0) throw std::invalid_argument("invalid servo calibration");
    pid_.initPid(kp,ki,kd,integral_limit,-integral_limit,true);
  }
  double update(double target,double position,double velocity,double dt,double feedforward=0.) {
    if (!std::isfinite(target) || !std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(dt) || !std::isfinite(feedforward) || dt<=0 || dt>.02) throw std::invalid_argument("invalid servo feedback/period");
    // D on measured velocity avoids derivative kick on position setpoint steps
    // and finite-difference angle noise. This is torque actuation, no teleport.
    const double effort=feedforward+pid_.computeCommand(target-position,-velocity,
        static_cast<uint64_t>(dt*1e9));
    if (!std::isfinite(effort)) throw std::runtime_error("nonfinite servo effort");
    return std::clamp(effort,-limit_,limit_);
  }
 private:
  control_toolbox::Pid pid_;
  double limit_;
};
}
