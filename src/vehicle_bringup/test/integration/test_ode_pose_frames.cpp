#include "ode_pose_frames.hpp"
#include <limits>
#include <iostream>

void append(gazebo::msgs::PosesStamped &message, const std::string &name,
            const ignition::math::Pose3d &pose) {
  auto *p = message.add_pose(); p->set_name(name);
  gazebo::msgs::Set(p, pose);
}
int main() {
  using ignition::math::Pose3d;
  gazebo::msgs::PosesStamped message;
  const Pose3d model(1,2,3,.02,-.03,.7), nested(-.2,.4,.5,.1,.2,-.3), link(.1,.2,.3,-.4,.3,.2);
  append(message,"robot::sub::link",link); // Arbitrary publication order.
  append(message,"robot",model);
  append(message,"robot::sub",nested);
  Pose3d actual;
  const auto equal=[](const Pose3d &a,const Pose3d &b) {
    return (a.Pos()-b.Pos()).Length()<1e-12 &&
           (a.Rot().Inverse()*b.Rot()).Euler().Length()<1e-12;
  };
  if (!resolveWorldPose(message,"robot::sub::link",actual) || !equal(actual,model*nested*link)) return 1;
  if (!resolveWorldPose(message,"robot",actual) || !equal(actual,model)) return 2;
  if (resolveWorldPose(message,"missing::link",actual)) return 3;
  gazebo::msgs::PosesStamped incomplete;
  append(incomplete,"robot::link",link);
  if (resolveWorldPose(incomplete,"robot::link",actual)) return 4;
  append(incomplete,"robot",model);
  incomplete.mutable_pose(0)->mutable_orientation()->set_w(std::numeric_limits<double>::quiet_NaN());
  if (resolveWorldPose(incomplete,"robot::link",actual)) return 5;
  std::cout << "world composition, nested order, missing-parent and invalid-pose checks PASS\n";
}
