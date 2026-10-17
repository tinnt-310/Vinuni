#include <gazebo/ode/ode.h>
#include <cmath>
#include <iostream>

namespace {
struct Observation {double depth, speed, force;};
Observation settle(double step, double correction_cap) {
  const auto world=dWorldCreate();
  const auto space=dSimpleSpaceCreate(nullptr);
  const auto joints=dJointGroupCreate(0);
  dWorldSetGravity(world,0.,0.,-50.); // Independent 0.1 kg /5 N load.
  dWorldSetQuickStepNumIterations(world,100);
  dWorldSetQuickStepW(world,1.);
  dWorldSetCFM(world,0.);
  dWorldSetContactSurfaceLayer(world,0.);
  dWorldSetContactMaxCorrectingVel(world,correction_cap);
  const auto body=dBodyCreate(world);
  dBodySetAutoDisableFlag(body,0);
  dMass mass;dMassSetSphereTotal(&mass,.1,.01);dBodySetMass(body,&mass);
  dBodySetPosition(body,0.,0.,.01);
  const auto sphere=dCreateSphere(space,.01);dGeomSetBody(sphere,body);
  const auto plane=dCreatePlane(space,0.,0.,1.,0.);
  dJointFeedback feedback{};
  double force=0.;
  for(unsigned tick=0;tick<static_cast<unsigned>(1./step);++tick) {
    dContact contact{};
    if(dCollide(sphere,plane,1,&contact.geom,sizeof(contact))) {
      // Gazebo combines two20 kN/m surfaces harmonically, two100 Ns/m
      // dampers add. Keep the actual matched calibration, no rigid-contact
      // substitute. ODE cap limits the ERP correction velocity.
      constexpr double stiffness=10000., damping=200.;
      contact.surface.mode=dContactSoftERP|dContactSoftCFM;
      contact.surface.soft_erp=step*stiffness/(step*stiffness+damping);
      contact.surface.soft_cfm=1./(step*stiffness+damping);
      const auto joint=dJointCreateContact(world,joints,&contact);
      dJointAttach(joint,body,nullptr);dJointSetFeedback(joint,&feedback);
    }
    dWorldQuickStep(world,step);
    force=feedback.f1[2];
    dJointGroupEmpty(joints);
  }
  Observation result{.01-dBodyGetPosition(body)[2],dBodyGetLinearVel(body)[2],force};
  dJointGroupDestroy(joints);dSpaceDestroy(space);dWorldDestroy(world);
  return result;
}
}

int main() {
  dInitODE2(0);bool ok=true;
  for(double step:{.000125,.0000625}) {
    const auto clipped=settle(step,.01), calibrated=settle(step,.1);
    // With two surfaces,0.01*(200+h*10000) only supplies~2 N at rest.
    // A5 N load therefore keeps sinking; timestep refinement cannot fix it.
    ok &= clipped.depth>.0005 && clipped.speed<-.01;
    // QuickStep approaches the analytic5 N/k equilibrium with its solver
    // residual. Require the unchanged500 um penetration gate and load within
    // the same0.1 N resolution used by the existing physical ODE fixtures.
    ok &= calibrated.depth<=.0005 && std::abs(calibrated.depth-.0005)<1e-5 &&
          std::abs(calibrated.speed)<1e-4 && std::abs(calibrated.force-5.)<.1;
    std::cout<<"step="<<step<<" clipped depth="<<clipped.depth<<" speed="<<clipped.speed
             <<" calibrated depth="<<calibrated.depth<<" force="<<calibrated.force<<'\n';
  }
  dCloseODE();return ok?0:1;
}
