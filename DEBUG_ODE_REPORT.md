# DEBUG ODE REPORT

## Symptom
Gazebo Classic ODE `LCP internal error, s <= 0`, including nonfinite s. Earlier action-success claims were insufficient. This report supersedes earlier stability claims.

## Reproduction procedure
ROS 2 Humble, isolated ROS_DOMAIN_ID=93 / Gazebo port 11393. Same Ready home [0,-pi/2,pi/2,-pi/2,-pi/2,0], target [.15,-pi/2,pi/2,-pi/2,-pi/2,.3], duration 4 s, zero final velocity, 2 s observation before command and 3 s hold after action. Same JointTrajectoryController position command, same vinfast_map.world and physics. Each case launches a fresh Gazebo process. Logs, expanded URDF, actual controller YAML and sampled position/velocity/effort live in docs/ode_debug/<case>/.

Run from workspace root in a fresh terminal:
```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --build-base build_grouped --install-base install_grouped --symlink-install --packages-select vehicle_description vehicle_bringup fr10_arm_control
source install_grouped/setup.bash
python3 tools/run_ode_regression.py repeat_1
python3 tools/run_ode_regression.py repeat_2
python3 tools/run_ode_regression.py repeat_3
```
The existing grouped build/install layout is retained; do not source the stale install/ from before package relocation. The runner checks LCP, numeric nonfinite tokens, joint-state finiteness, action result and final angular error, and exits nonzero on failure. It does not connect to real hardware.

## Tests performed and results
Order: A collision isolation first, with B/C static validation in parallel. No solver tuning in this debug pass. Success state status=4.

| Test | LCP count | Action status | All joint fields finite | Max final angle error rad |
|---|---:|---:|---|---:|
| base_clearance | 0 | 4 | False | 9.66445e-06 |
| baseline | 552 | 4 | False | 9.67587e-06 |
| final_1 | 0 | 4 | True | 9.66445e-06 |
| final_2 | 0 | 4 | True | 9.66445e-06 |
| final_3 | 0 | 4 | True | 9.66445e-06 |
| no_collision | 6 | 4 | False | 9.66445e-06 |
| no_tool_collision | 579 | 4 | False | 7.72653e-06 |
| static_dumpster | 593 | 4 | False | 1.87185e-05 |
| static_no_base_collision | 0 | 4 | False | 9.66445e-06 |
| vehicle_final_1 | 331 | 4 | False | 9.66599e-06 |
| vehicle_no_arm_collision | 263 | 4 | False | 9.66599e-06 |
| vehicle_no_wheel_collision | 3707 | 4 | False | 9.78107e-06 |

## Root cause established for standalone FR10
1. World Dumpster was dynamic (mass 1 kg, inertia diagonal 1, mesh scale 1.5) with mesh-ground contact. Baseline emits errors before spawn_entity calls the robot spawn service. Changing only this obstacle to static removes pre-spawn errors, but does not remove the arm errors.
2. fr10_base_link is fixed to world at z=0 and its vendor mesh collision also touches the ground. Disabling just this base collision while keeping every arm/tool collision eliminates LCP. Replacing it with a cylindrical collision (radius .095 m, top .0848 m, bottom .001 m) eliminates the redundant ground contact without changing visuals, kinematics or disabling collision. Same test then passes three times.
3. NaN effort fields in joint_states were unregistered measurements: the ros2_control block exported position/velocity only. Adding an effort state interface gives finite values. Gazebo position-command backend reports zero effort here; this is not a validated motor torque estimate and should not be used for load sizing.

## Collision audit
Expanded SDF fixed-joint lumping attaches plate, D435, CCS2 and brackets to wrist3_link (12 collisions total including wrist); those rigidly attached geometries do not represent separate dynamic bodies colliding with each other. Removing their collisions did not remove the error. Their visual poses, collision poses, masses and inertia remain unchanged. Wrist3/plate, plate/camera, plate/charger and camera/charger therefore were excluded as the cause in this reproduction. Removing all arm collisions on the vehicle did not resolve that separate coupled-system failure. No diagnostic collision removal remains in the production launch.

## Dynamics, joint and controller checks
All expanded inertial tensors have positive eigenvalues, positive masses, satisfy principal-moment triangle inequalities and are finite; checked both URDF and post-lumping SDF. See inertia_audit.json. Some vendor COM locations remain outside visible mesh bounds; not replaced because the controlled collision test solved standalone reproduction without changing them. Home and targets are inside all six hard limits, including j6 +/-3.0543 rad. No limit, command interface, gain, effort limit or desired pose changed. Position mode has no JTC effort PID to tune. Controller measured state was recorded throughout the same test, not inferred from action success.

## Files modified / old and new values
- vehicle_bringup/launch/simulation.launch.py: Gazebo runtime base collision mesh -> cylinder radius .095, length .0838, origin [0,0,.0429]. Only collision modified; manufacturer source/visual unchanged.
- vehicle_description/vehicle_description/worlds/vinfast_map.world: Dumpster dynamic -> static true, retains its visual and collision.
- fr10_arm_control/fr10_arm_control/config/ros2_control.xacro: add effort state interface on j1..j6, retain position commands.
- tools/ode_repro_client.py and tools/run_ode_regression.py: repeatable regression and failure gate.

ODE solver remains world, dt .001 s, CFM 1e-5, ERP .2; these were pre-existing at the start of this debug pass, not the fix. Chassis mass remains 150 kg. Tool transforms and Ready pose unchanged.

## Final validation and remaining warnings
Standalone final_1, final_2, final_3: 0 LCP, no numeric NaN/Inf log tokens, all joint fields finite, action succeeded, max final joint error <1e-5 rad. Build passed. This validates the fixed test, not every reachable trajectory. Inspect WARN lines in per-case logs (controller update period vs physics period and occasional DDS service timeout messages); no error suppression is applied.

IMPORTANT: vehicle_final_1 still produced LCP. vehicle_no_arm_collision did not resolve it; vehicle_no_wheel_collision worsened it as the un-supported chassis fell. That last diagnostic is NOT evidence for deleting wheel collisions. These temporary cases do not modify production sources. The coupled vehicle/ground-contact case remains unaccepted; standalone success must not be advertised as a complete vehicle fix. No new vehicle solver or physics workaround has been committed in this pass.
