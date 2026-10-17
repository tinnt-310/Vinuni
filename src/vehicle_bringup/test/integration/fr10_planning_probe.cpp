// Read-only planning/trajectory probe. No actuator publisher or SDK dependency.
#include <fstream>
#include <cstring>
#include <iterator>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "fr10_pose_controller/node_parameters.hpp"
#include "fr10_motion_planning/moveit_ik_provider.hpp"
#include "fr10_motion_planning/collision_validated_ik_provider.hpp"
#include "fr10_motion_planning/joint_route_planner.hpp"
#include "fr10_motion_planning/online_motion_validator.hpp"
#include "fr10_motion_core/continuous_ik_stepper.hpp"
#include "fr10_motion_core/math_utils.hpp"

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("fr10_pose_controller");
  auto params=fr10_pose_controller::NodeParameters::declare(node.get());
  auto scene_path=node->declare_parameter<std::string>("probe.scene_file","");
  auto output=node->declare_parameter<std::string>("probe.output_file","");
  auto q=node->declare_parameter<std::vector<double>>("probe.initial_joint_rad",std::vector<double>{});
  auto p=node->declare_parameter<std::vector<double>>("probe.mouth_position_m",std::vector<double>{});
  auto orientation=node->declare_parameter<std::vector<double>>("probe.orientation_xyzw",std::vector<double>{});
  double approach=node->declare_parameter<double>("probe.approach_m",.03);
  double depth=node->declare_parameter<double>("probe.depth_m",.015);
  if(q.size()!=6 || p.size()!=3 || orientation.size()!=4 || output.empty())return 2;
  robot_model_loader::RobotModelLoader loader(node,"robot_description",false);
  auto model=loader.getModel();
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  std::ifstream file(scene_path,std::ios::binary);
  std::vector<char> bytes((std::istreambuf_iterator<char>(file)),{});
  if(bytes.empty())return 2;
  rclcpp::SerializedMessage data(bytes.size());
  auto &serialized=data.get_rcl_serialized_message();
  std::memcpy(serialized.buffer,bytes.data(),bytes.size());serialized.buffer_length=bytes.size();
  moveit_msgs::msg::PlanningScene message;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&data,&message);
  scene->setPlanningSceneDiffMsg(message);
  auto &allowed=scene->getAllowedCollisionMatrixNonConst();
  allowed.setEntry("charging_port_mating_surfaces","charging_mating_nose",true);
  allowed.setEntry("charging_port_mating_surfaces","charging_mating_shoulder",true);
  auto &state=scene->getCurrentStateNonConst();
  auto group=model->getJointModelGroup(params.kinematics_group);
  state.setJointGroupPositions(group,q);state.update();
  fr10_motion_planning::MoveItIKProviderConfig config;
  config.group_name=params.kinematics_group;config.tip_link=params.kinematics_tip_link;
  config.base_link=params.base_frame;config.safety_link=params.kinematics_safety_link;
  config.limits=params.controller_config.limits;config.wrist_min_abs_rad=params.wrist_singularity_min_rad;
  config.elbow_min_abs_rad=params.elbow_singularity_min_rad;config.max_wrist_reach_m=params.max_wrist_reach_m;
  config.min_wrist_radius_m=params.min_wrist_radius_m;config.max_jacobian_condition=params.max_jacobian_condition;
  config.collision_checking_enabled=true;config.planning_scene=scene;
  fr10_motion_planning::MoveItIKProvider validator(model,config);validator.setPlanningSceneSnapshot(scene);
  auto analytical_config=params.analytical_ik_config;
  const Eigen::Isometry3d tool=state.getGlobalLinkTransform("wrist3_link").inverse()*state.getGlobalLinkTransform(params.kinematics_tip_link);
  Eigen::Quaterniond rotation(tool.rotation());
  for(unsigned i=0;i<3;++i)analytical_config.wrist3_to_tip.position_m[i]=tool.translation()[i];
  analytical_config.wrist3_to_tip.orientation_quat={rotation.x(),rotation.y(),rotation.z(),rotation.w()};
  fr10_motion::Fr10AnalyticalIKProvider analytical(analytical_config);
  fr10_motion_planning::CollisionValidatedIKProvider provider(analytical,validator);
  fr10_motion::KinematicState seed;std::copy(q.begin(),q.end(),seed.position_rad.begin());
  fr10_motion::Pose mouth;std::copy(p.begin(),p.end(),mouth.position_m.begin());
  std::copy(orientation.begin(),orientation.end(),mouth.orientation_quat.begin());
  auto axis=fr10_motion::math::rotateVector(mouth.orientation_quat,{0.,0.,1.});
  auto preinsert=mouth;
  for(unsigned i=0;i<3;++i)preinsert.position_m[i]-=axis[i]*approach;
  // Static nominal corridor and slow forward/reverse stopping screening. This
  // probe proves reachability/trajectory validity, not physical contact/HOLD.
  auto guard=[&](const fr10_motion::KinematicState &endpoint,const auto &cancel) {
    fr10_motion::ContinuousIKStepper stepper(params.continuous_ik_config);
    auto previous=endpoint;
    unsigned count=static_cast<unsigned>(std::ceil((approach+depth+.0045)/.001));
    for(unsigned index=0;index<=count;++index) {
      if(cancel())return false;
      auto target=mouth;double distance=-approach-.002+(approach+depth+.0045)*index/count;
      for(unsigned i=0;i<3;++i)target.position_m[i]+=axis[i]*distance;
      auto next=stepper.step(target,previous,provider);if(!next.success)return false;
      if(index>0)for(double sign:{-1.,1.}) {
        auto braking=next.state;
        for(unsigned j=0;j<6;++j)braking.velocity_rad_s[j]=sign*.0006*
            (next.state.position_rad[j]-previous.position_rad[j])/((approach+depth+.0045)/count);
        if(!fr10_motion_planning::validateControlledStop(braking,provider,config.limits,
            params.nominal_period_s,params.controller_config.max_joint_reference_lead_rad,cancel))return false;
      }
      previous=next.state;
    }
    return true;
  };
  auto start=std::chrono::steady_clock::now();
  auto route=fr10_motion_planning::JointRoutePlanner::plan(preinsert,seed,provider,params.cartesian_route_config,guard);
  YAML::Node report;report["invariant_clear_pairs"]=validator.invariantClearPairCount();report["route_success"]=route.success;report["route_message"]=route.message;
  report["detour"]=route.used_detour;
  report["route_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  report["pass"]=false;report["stage"]="route_materialized";
  report["waypoints"]=route.cartesian_path.size();
  std::ofstream(output)<<report<<'\n';
  bool valid=route.success;
  if(valid) {
    auto timed=fr10_motion::PathTimeParameterizer::parameterize(route.cartesian_path,route.continuous_ik.joint_path,
        seed,params.path_time_policies[0],1,{},[&](const auto &q,auto &pose){return analytical.forward(q,pose);});
    valid=timed.success;report["timing_success"]=timed.success;report["timing_message"]=timed.message;
    report["trajectory_duration_s"]=timed.duration_s;report["samples"]=timed.trajectory.points.size();
    report["scaling_iterations"]=timed.diagnostics.scaling_iterations;
    report["max_linear_jerk_m_s3"]=timed.max_linear_jerk_m_s3;
    report["max_angular_jerk_rad_s3"]=timed.max_angular_jerk_rad_s3;
    report["stage"]="timed_collision_validation";
    std::ofstream(output)<<report<<'\n';
    auto previous=seed;double maximum_condition=0.;
    for(const auto &point:timed.trajectory.points) {
      auto checked=provider.validate(point.joint_state,previous);
      if(!checked.success){valid=false;report["sample_rejection"]=checked.message;break;}
      maximum_condition=std::max(maximum_condition,checked.condition_number);previous=point.joint_state;
    }
    report["trajectory_duration_s"]=timed.duration_s;report["samples"]=timed.trajectory.points.size();
    report["maximum_condition"]=maximum_condition;
    report["velocity_ratio"]=timed.diagnostics.max_joint_velocity_ratio;
    report["acceleration_ratio"]=timed.diagnostics.max_joint_acceleration_ratio;
    report["jerk_ratio"]=timed.diagnostics.max_joint_jerk_ratio;
  }
  report["stage"]="complete";report["pass"]=valid;report["actuation"]=false;report["physical_contact_acceptance"]=false;
  std::ofstream(output)<<report<<'\n';rclcpp::shutdown();return valid?0:1;
}
