#include <iostream>
#include <cmath>
#include "fr10_motion_planning/joint_route_planner.hpp"

using namespace fr10_motion;

class Fixture final : public IKProvider {
public:
  bool obstruction{true};
  bool height_escape{false};
  IKResult solve(const Pose &, const KinematicState &) override { return {}; }
  std::size_t solveCandidates(const Pose &goal, const KinematicState &,
      std::array<IKResult, kMaxCandidates> &results, std::string *) override {
    for (std::size_t i = 0; i < 2; ++i) {
      results[i].success = true;
      results[i].state.position_rad[0] = goal.position_m[0];
      results[i].state.position_rad[5] = 0.5 * i;
      results[i].state.position_rad[2] = goal.position_m[2];
    }
    return 2;
  }
  IKResult validate(const KinematicState &state, const KinematicState &) override {
    IKResult result;
    result.state = state;
    const auto &q = state.position_rad;
    result.success = !obstruction || !(std::abs(q[0]) < .25 &&
        (height_escape ? q[2]<.03 : q[1]<.4));
    result.condition_number = 1.;
    result.manipulability = 1.;
    return result;
  }
  bool forward(const KinematicState &state, Pose &pose, std::string *) const override {
    pose.position_m = {state.position_rad[0], state.position_rad[1], state.position_rad[2]};
    pose.orientation_quat = {0., 0., 0., 1.};
    return true;
  }
};

int main() {
  Fixture provider;
  CartesianRoutePlannerConfig config;
  config.path.max_waypoints = 5000;
  config.path.max_translation_step_m = .01;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    config.limits.min_joint_position_rad[j] = -2.;
    config.limits.max_joint_position_rad[j] = 2.;
  }
  KinematicState seed;
  seed.position_rad[0] = -1.;
  Pose goal{{1., 0., 0.}, {0., 0., 0., 1.}};
  auto guard = [](const KinematicState &endpoint, const auto &) { return endpoint.position_rad[5] > .4; };
  auto route = fr10_motion_planning::JointRoutePlanner::plan(goal, seed, provider, config, guard);
  if (!route.success || !route.used_detour ||
      route.continuous_ik.joint_path.back().position_rad[5] < .4) {
    std::cerr << "Detour failed: " << route.message << " direct="
              << route.direct_failure_message << '\n'; return 1;
  }
  for (const auto &q : route.continuous_ik.joint_path)
    if (!provider.validate(q, q).success) return 2;
  auto rejected = fr10_motion_planning::JointRoutePlanner::plan(
      goal, seed, provider, config, [](const auto &, const auto &) { return false; });
  if (rejected.success) return 3;
  auto canceled = fr10_motion_planning::JointRoutePlanner::plan(
      goal, seed, provider, config, guard, [] { return true; });
  if (canceled.success) return 4;
  provider.obstruction = false;
  auto direct = fr10_motion_planning::JointRoutePlanner::plan(goal, seed, provider, config, guard);
  if (!direct.success || direct.used_detour) return 5;
  unsigned checked = 0;
  auto bounded = fr10_motion_planning::JointRoutePlanner::plan(
      goal, seed, provider, config, [&](const auto &, const auto &budget_cancel) {
        while (!budget_cancel()) ++checked;
        return false;
      }, {}, .001);
  if (bounded.success || checked == 0) return 6;
  provider.obstruction=true;provider.height_escape=true;
  config.singularity_avoidance_enabled=true;config.detour_clearances_m={.10};
  auto clearance=fr10_motion_planning::JointRoutePlanner::plan(goal,seed,provider,config,guard);
  if(!clearance.success || !clearance.used_detour ||
      clearance.message.find("clearance waypoint")==std::string::npos)return 7;
  provider.obstruction=false;
  KinematicState a,b,c;b.position_rad[0]=1.;c.position_rad[0]=c.position_rad[1]=1.;
  Pose corner_goal{{1.,1.,0.},{0.,0.,0.,1.}};
  auto blended=CartesianRoutePlanner::fromJointWaypoints({a,b,c},corner_goal,provider,config);
  if(!blended.success || blended.message.find("blended")==std::string::npos)return 8;
  bool corner_trimmed=true;double minimum_step=1e9,maximum_step=0.;
  const auto &points=blended.continuous_ik.joint_path;
  for(std::size_t i=0;i<points.size();++i) {
    if(std::hypot(points[i].position_rad[0]-1.,points[i].position_rad[1])<.01)corner_trimmed=false;
    if(i) {
      double length=0.;for(unsigned j=0;j<6;++j)
        length=std::hypot(length,points[i].position_rad[j]-points[i-1].position_rad[j]);
      minimum_step=std::min(minimum_step,length);maximum_step=std::max(maximum_step,length);
    }
  }
  if(!corner_trimmed || maximum_step/minimum_step>1.01)return 9;
  provider.obstruction=true;
  KinematicState low_left,left_corner,right_corner,low_right;
  low_left.position_rad[0]=-1.;left_corner.position_rad[0]=-.30;left_corner.position_rad[1]=.40;
  right_corner.position_rad[0]=.30;right_corner.position_rad[1]=.40;low_right.position_rad[0]=1.;
  provider.height_escape=false;
  auto tight=CartesianRoutePlanner::fromJointWaypoints({low_left,left_corner,right_corner,low_right},goal,provider,config);
  if(!tight.success || tight.message.find("blended")!=std::string::npos)return 10;
  for(const auto &q:tight.continuous_ik.joint_path)if(!provider.validate(q,q).success)return 11;
  std::cout << "Obstacle detour, insertion branch selection, cancellation and direct route PASS\n";
  return 0;
}
