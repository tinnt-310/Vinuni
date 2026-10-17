# Vehicle LCP investigation — production candidate, acceptance pending

The production candidate now sets wheel ODE contact slip and vehicle spawn
height. Do not infer universal stability from one zero count; complete physics
and controller traces, repeated runs, and Docker remain required.

## Isolation evidence

The original ROS domain 93 contained three old gzserver processes. Their service
discovery could contaminate clients. The new probe uses domain 94 and port 11394;
each case launches a new server. After the later session restart no old gzserver
was present. Docker `fairino-container` previously had no mounted workspace;
these Gazebo tests run on the host with `install_grouped`, not in that container.

`vehicle_isolated_v0_baseline_20260926`: 12,940 LCP, first at steady-clock
23938.74806782. Last controller response before it: empty controller list.
Only four wheel/ground contact pairs were present around the first failure.
Thus active controller commands are not necessary to trigger the passive fault.

`vehicle_isolated_v0_direction_20260926`: 12,863 LCP. At first LCP, simulation
time 184.239, the four world-frame friction directions had normal components
0.0110703, -0.0196621, -0.0124202, 0.0201773 rather than zero. The configured
collision-local `fdir1=1 0 0` rotates with wheel roll. This is a contact-direction
defect, not evidence of invalid mass or an FR10 command.

Gazebo's source rotates this vector to world coordinates and passes it to ODE:
[ODEPhysics.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc).
The installed header documents friction direction relative to the collision.

Changing only fdir1 to automatic (`0 0 0`) eliminates LCP in one isolated passive
run (`vehicle_isolated_v0_auto_20260926`), but is **not a complete fix**.
It also changes the orientation of anisotropic friction and cannot be accepted
as production-equivalent without validating motion behavior.

## Resumed dynamic experiments

All below use original solver, spawn z=0.1, kp, mass and geometry; only fdir1 is
temporarily automatic in generated launch files.

| Evidence directory under docs/ode_debug | Case | LCP | Measured outcome |
|---|---|---:|---|
| vehicle_isolated_v5_auto_resume | V5 forward | 1 | Actual displacement [2.249907, 0.082416] m; finite physics/ROS; FAIL |
| vehicle_isolated_v4_auto_resume2 | V4 active zero | 0 | 27.94 s measured model state; finite; max final speed 4.05e-10 m/s; diagnostic gate PASS |
| vehicle_isolated_v7_auto_resume2 | V7 rotate | 2180 | Yaw change 2.279645 rad; finite physics/ROS; FAIL |
| vehicle_isolated_v6_auto_resume | V6 lateral | 0 | INVALID: physics trace truncated after 220 complete lines; spawn stalled |

V5 first LCP was ~0.21 s after drive activation, before the commanded motion.
V7 first LCP was simulation time 206.338 near the transition to stopped wheels.
Finite chassis state and a small final velocity do not override the zero-LCP gate.

## Instrumentation changes and limitations

- `tools/ode_contact_probe.cpp`: records actual chassis pose/velocity, joint
  position/velocity/applied force, contacts, depths, normals and contact forces.
  Now a model plugin, using UpdateInfo time instead of synchronous world queries
  in the physics callback. The earlier world-plugin traces above remain evidence
  only when complete; the new instrumentation needs its own fresh validation.
- `tools/run_vehicle_probe.py`: V0–V8 controller isolation, fresh launches,
  expanded URDF/controller YAML and timestamped raw logs. No production changes.
- `tools/vehicle_probe_observer.py`: ROS state/controller/command trace; publishes
  test commands at no more than 50 Hz. Earlier forward test used an unbounded
  publish loop; repeat with the corrected harness before acceptance.
- `tools/analyze_vehicle_probe.py`: checks actual physics state and motion, finite
  data, controller coverage and final speed; it deliberately does not equate
  odometry movement with chassis movement.

Log arrival timestamps are not exact ODE solver call timestamps. The first-LCP
window is a nearby physics trace, not proof of the exact failing constraint row.

## Controlled contact compliance experiment

The original-fdir1 V0 passive baseline gave 12,940 LCP with no controllers
active. Adding only ODE surface slip1=slip2=1e-5 gave 0 LCP in a new passive
run, though its 18.47 s model trace is shorter than the final gate. Original
fdir1 with that same slip gave 0 LCP in 27.84 s of vehicle rotation; actual
yaw changed 2.233 rad. With automatic fdir1, slip reduced V7 from 2,180 to
0 LCP in two new runs. V5, V6 and V8 also gave 0 LCP with actual chassis
motion, finite state and stop. The exact failing ODE row is still unknown;
the controlled observations isolate rigid tangential wheel contacts as the
operational trigger. The earlier Xacro slip experiment was invalid because
URDF-to-SDF conversion placed slip tags outside the collision surface.

The production candidate is src/vehicle_bringup/src/wheel_contact_surface.cpp.
It sets ODE slip1 and slip2 to 1e-5 for the four wheel collision surfaces,
leaving original fdir1, friction coefficients, gains, world solver, masses,
motion limits and commands unchanged. It is loaded only for vehicle models.
The launch spawn height was separately corrected from 0.1 m to 0.0 m after
expanded geometry and measured free-fall drift confirmed wheel/ground contact
at base_footprint z=0. A direct probe read all four runtime ODE surfaces as
[1e-5, 1e-5]. The V0–V8 matrix and primary vehicle/mounted runs passed once.

## Next controlled checks

The nine V0–V8 measured results under docs/ode_debug/vehicle_final_matrix_V*_20260927 each show zero LCP over about 28.6–29.0 s. [Vehicle standalone](docs/ode_debug/vehicle_ros_pose_probe_20260927/summary.json) ran 31.61 s with 0 LCP and 0.752 m actual displacement; [mounted coupled](docs/ode_debug/mounted_coupled_probe_20260927/summary.json) ran 37.12 s with 0 LCP, 1.207 m actual chassis displacement and 4.05 s overlap between moving arm and vehicle. Raw logs and expanded generated artifacts are in the same directories.

Long lateral travel in the production world reaches the angled practice barrier. First barrier contact preceded the first LCP by about 0.51 s. Removing only that barrier in a generated diagnostic world yielded 0 LCP during 43 s and 4.46 m actual lateral travel. This isolates a separate obstacle-contact instability; production world remains unchanged. The exact ODE failing constraint row is still unobserved.

Three fresh controller-gated host-Gazebo repetitions pass for each primary scenario: arm, vehicle and mounted, with 0 LCP and no server crash in all nine summaries ([main report](FR10_AUDIT_REGRESSION_REPORT.md) §10). Three measured 45-second V0–V8 matrix repetitions also have 27/27 valid PASS results, 0 LCP and finite ROS/physics state; representative [third-run V0](docs/ode_debug/vehicle_transport_matrix3_ready_V0_20260927/measured_result.json), [V5](docs/ode_debug/vehicle_transport_matrix3_V5_20260927/measured_result.json), and [V7](docs/ode_debug/vehicle_transport_matrix3_V7_20260927/measured_result.json). The initial third-run V0 attempt separately crashed in `gzserver` with exit -11 immediately after robot spawn, before any model-state sample; its [raw log](docs/ode_debug/vehicle_transport_matrix3_V0_20260927/gazebo.log) is retained. The successful rerun does not erase this startup-reliability defect.

These free-space results do not close the angled-barrier impact instability or Docker/Gazebo gate. One earlier `gzserver` service-response abort remains unexplained. The contact change remains a production candidate for the measured free-space cases, not a claim of universal stability; the exact ODE failing constraint row remains unobserved.
