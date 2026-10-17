#include "fr10_motion_planning/joint_route_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace fr10_motion_planning {
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace fr10_motion;

CartesianRoutePlan JointRoutePlanner::plan(
    const Pose &goal, const KinematicState &seed, IKProvider &provider,
    const CartesianRoutePlannerConfig &config, const EndpointGuard &endpoint_guard,
    const CartesianRoutePlanner::CancelCheck &cancel, double search_timeout_s) {
  CartesianRoutePlan failure;
  if (!std::isfinite(search_timeout_s) || search_timeout_s <= 0.0) {
    failure.message = "invalid planner budget";
    return failure;
  }
  const auto initial = provider.validate(seed, seed);
  if (!initial.success) {
    failure.message = "invalid initial robot state: " + initial.message;
    return failure;
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(search_timeout_s);
  const CartesianRoutePlanner::CancelCheck bounded_cancel = [&]() {
    return std::chrono::steady_clock::now() >= deadline || (cancel && cancel());
  };
  std::array<IKResult, IKProvider::kMaxCandidates> candidates{};
  const auto count = provider.solveCandidates(goal, seed, candidates, &failure.message);
  std::vector<std::pair<double, KinematicState>> ranked;
  for (std::size_t i = 0; i < count; ++i) {
    if (bounded_cancel()) return failure;
    const auto &q = candidates[i].state;
    const auto validation = provider.validate(q, q);
    if (!validation.success || !std::isfinite(validation.condition_number)) continue;
    double sum = 0.0;
    for (std::size_t j = 0; j < kJointCount; ++j) {
      const double range = config.limits.max_joint_position_rad[j] -
                           config.limits.min_joint_position_rad[j];
      const double d = (q.position_rad[j] - seed.position_rad[j]) / range;
      sum += d * d;
    }
    ranked.emplace_back(sum + 0.01 * validation.condition_number, q);
  }
  // Collision/Jacobian checks are bounded worker operations, never sort
  // comparator side effects. Evaluate each branch quality exactly once.
  std::stable_sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });
  std::vector<KinematicState> eligible;
  for (const auto &ranked_endpoint : ranked) {
    const auto &endpoint = ranked_endpoint.second;
    if (bounded_cancel()) return failure;
    // Check in ranked order and stop once a complete direct route succeeds.
    // Screening all alternative insertion corridors can dominate planning.
    if (endpoint_guard && !endpoint_guard(endpoint, bounded_cancel)) continue;
    eligible.push_back(endpoint);
    auto direct = CartesianRoutePlanner::fromJointWaypoints(
        {seed, endpoint}, goal, provider, config, bounded_cancel);
    if (direct.success) return direct;
    failure = std::move(direct);
  }
  const auto &endpoints = eligible;
  if (endpoints.empty() || bounded_cancel()) {
    failure.message = "no IK branch supports the complete insertion segment";
    return failure;
  }
  // A small deterministic clearance waypoint often escapes a nearby fixture
  // faster than a six-dimensional random search. Its two full joint legs
  // receive the same dense FK/collision checks as every other candidate.
  Pose start_pose;
  if (provider.forward(seed,start_pose) && config.singularity_avoidance_enabled) {
    for(double clearance:config.detour_clearances_m) {
      if(bounded_cancel())return failure;
      auto staging=goal;
      for(std::size_t i=0;i<2;++i)
        staging.position_m[i]=.5*(start_pose.position_m[i]+goal.position_m[i]);
      staging.position_m[2]=std::max(start_pose.position_m[2],goal.position_m[2])+clearance;
      std::array<IKResult,IKProvider::kMaxCandidates> intermediate{};
      const auto n=provider.solveCandidates(staging,seed,intermediate,nullptr);
      for(std::size_t i=0;i<n;++i)for(const auto &endpoint:endpoints) {
        if(bounded_cancel())return failure;
        if(!intermediate[i].success)continue;
        auto detour=CartesianRoutePlanner::fromJointWaypoints(
            {seed,intermediate[i].state,endpoint},goal,provider,config,bounded_cancel);
        if(detour.success) {
          detour.message="clearance waypoint joint route densely validated";
          return detour;
        }
      }
    }
  }
  auto space = std::make_shared<ob::RealVectorStateSpace>(kJointCount);
  ob::RealVectorBounds bounds(kJointCount);
  for (std::size_t j = 0; j < kJointCount; ++j) {
    bounds.setLow(j, config.limits.min_joint_position_rad[j]);
    bounds.setHigh(j, config.limits.max_joint_position_rad[j]);
  }
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  auto decode = [](const ob::State *state) {
    KinematicState result;
    const auto *values = state->as<ob::RealVectorStateSpace::StateType>();
    for (std::size_t j = 0; j < kJointCount; ++j)
      result.position_rad[j] = values->values[j];
    return result;
  };
  setup.setStateValidityChecker([&](const ob::State *state) {
    if (bounded_cancel()) return false;
    const auto q = decode(state);
    return provider.validate(q, q).success;
  });
  // OMPL subdivision distance is Euclidean; this bounds every joint step.
  // Search coarsely, then materialize densely and refine only a rejected
  // route. A worst-case TCP bound at every random edge can consume the whole
  // budget before even one RRT vertex is added.
  double resolution = std::min(0.01,.5*config.joint_space_step_rad/space->getMaximumExtent());
  setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);
  ob::ScopedState<> start(space);
  for (std::size_t j = 0; j < kJointCount; ++j) start[j] = seed.position_rad[j];
  setup.setStartState(start);
  auto goals = std::make_shared<ob::GoalStates>(setup.getSpaceInformation());
  for (const auto &endpoint : endpoints) {
    ob::ScopedState<> state(space);
    for (std::size_t j = 0; j < kJointCount; ++j) state[j] = endpoint.position_rad[j];
    goals->addState(state.get());
  }
  goals->setThreshold(1e-9);
  setup.setGoal(goals);
  auto planner=std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
  planner->setRange(std::max(config.joint_space_step_rad,
      std::min(.3,10.*config.joint_space_step_rad)));
  setup.setPlanner(planner);
  const ob::PlannerTerminationCondition termination([&]() {
    return bounded_cancel();
  });
  while (!bounded_cancel()) {
    if (setup.solve(termination) != ob::PlannerStatus::EXACT_SOLUTION) break;
    auto &path = setup.getSolutionPath();
    if (!bounded_cancel()) setup.simplifySolution(termination);
    std::vector<KinematicState> route;
    route.reserve(path.getStateCount());
    for (const auto *state : path.getStates()) route.push_back(decode(state));
    route.front() = seed;
    auto result = CartesianRoutePlanner::fromJointWaypoints(route, goal, provider, config, bounded_cancel);
    if (result.success) {
      result.message = "RRTConnect intermediate joint route validated";
      return result;
    }
    failure = std::move(result);
    // A coarse collision sampler can miss a narrow obstruction. Never use
    // that route, but use the remaining budget to refine and search again.
    resolution *= .5;
    setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);
    setup.getPlanner()->clear();
    setup.getProblemDefinition()->clearSolutionPaths();
  }
  failure.message = "no densely validated collision/singularity-free route within planning budget: " + failure.message;
  return failure;
}
} // namespace fr10_motion_planning
