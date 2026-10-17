#pragma once

#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include "fr10_simulation_hardware/contact_wrench.hpp"

namespace fr10_simulation {

// Gazebo 11 ODEPhysics stores feedback in collision-link frames. ODE's
// dJointAttach puts a sole dynamic body in slot 1, even when collision 1 is
// static. Gazebo does not remap that slot: body1Force/body1Torque then hold
// the force on collision 2, expressed in collision 1's frame; slot 2 is not
// written by QuickStep. Never read the nonexistent static-body feedback.
inline void accumulateOdeToolContact(const gazebo::physics::Contact &contact,
    bool first_tool, const ignition::math::Vector3d &world_tcp,
    ContactWrenchAccumulator &sum) {
  if (!contact.collision1 || !contact.collision2 || contact.count < 0 ||
      contact.count > MAX_CONTACT_JOINTS) { sum.invalidate(); return; }
  const auto first = contact.collision1->GetLink();
  const auto second = contact.collision2->GetLink();
  const auto first_body = boost::dynamic_pointer_cast<gazebo::physics::ODELink>(first);
  const auto second_body = boost::dynamic_pointer_cast<gazebo::physics::ODELink>(second);
  if (!first_body || !second_body ||
      !(first_tool ? first_body : second_body)->GetODEId()) {
    sum.invalidate(); return;
  }
  // Detection-only overlaps introduce no physical constraint or contact load.
  if (contact.collision1->GetSurface()->collideWithoutContact ||
      contact.collision2->GetSurface()->collideWithoutContact) return;
  const bool first_slot = first_tool || !first_body->GetODEId();
  const auto source = first_slot ? first : second;
  const auto tool = first_tool ? first : second;
  const auto rotation = source->WorldPose().Rot();
  const auto com = tool->WorldCoGPose().Pos();
  const auto vector = [](const ignition::math::Vector3d &v) -> Eigen::Vector3d {
    return {v.X(), v.Y(), v.Z()};
  };
  for (int j = 0; j < contact.count; ++j) {
    const auto &load = contact.wrench[j];
    sum.add(vector(rotation.RotateVector(first_slot ? load.body1Force : load.body2Force)),
        vector(rotation.RotateVector(first_slot ? load.body1Torque : load.body2Torque)),
        vector(com), vector(world_tcp));
  }
}

} // namespace fr10_simulation
