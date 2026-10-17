# FR10 audit, refactor and Gazebo regression — 2026-09-26

Status as of 2026-09-27: **incomplete** against the full acceptance criteria. Three controller-gated arm/vehicle/mounted runs and three measured V0–V8 host-Gazebo matrices passed with zero ODE LCP errors; the latest after-controller-edit rerun also passed all three scenarios. One additional V0 attempt crashed during Gazebo startup and is retained as a failure. Full `motion_adapter.cpp` decomposition, whole-workspace per-file audit, intermittent feedback-dropout isolation, obstacle-impact stability and Docker/Gazebo execution remain outstanding. Sections 4–9 retain historical failures; sections 10–14 record newer evidence.

## 1. Architecture and `fr10_pose`/`motion_adapter` conclusion

Baseline inventory, per-file responsibilities, dependencies, algorithms and proposed boundaries are in [AUDIT_REPORT.md](AUDIT_REPORT.md). In brief:

```text
Before and currently: ROS input -> motion_adapter.cpp (validation, policy,
planning worker, activation, safety monitoring, diagnostics, lifecycle)
-> MotionExecutor -> RobotBackend -> RosControlBackend -> ros2_control

Target: ROS input -> pure command validation -> pose policy coordinator
-> scene-aware planning service -> MotionExecutor -> RobotBackend
-> ros2_control/GazeboSystem or FAIRINO hardware plugin
```

The `fr10_pose_controller` package already has a ROS-independent motion core, a `RobotBackend` interface, `RosControlBackend`, MoveIt providers and the sole FAIRINO SDK hardware plugin. These are sound layer boundaries. `motion_adapter.cpp` was 2,741 lines before this pass and contains approximately 30 methods spanning command decoding, worker planning, insertion policy, 125 Hz execution, telemetry, signal handling and ROS lifecycle. That is a God file by responsibility. Git history contains only one initial commit, already after the split, and no old `fr10_pose` source; the number of copied lines cannot be proved. The `AGENTS.md` master-specification path is also absent.

This pass extracted one duplicated Cartesian tracking calculation into `cartesian_tracking.hpp`; both the safety monitor and diagnostics now call the same pure function. The function returns infinity for nonfinite input so the fault monitor can fail closed. It has a ROS-free unit test. This is a bounded refactor, not the requested full decomposition of `motion_adapter.cpp`. No public ROS topic, limit, driver route or SDK call was changed.

The Gazebo regression graph is separate from `motion_adapter.cpp`: `vehicle_bringup/simulation.launch.py` starts `gazebo_ros2_control/GazeboSystem`, `joint_trajectory_controller` and swerve controllers. It does not launch `fr10_pose_controller_node`. Therefore these Gazebo LCP logs cannot be directly caused by `motion_adapter.cpp` output. They do establish a physical-model/solver problem that must be resolved for acceptance.

## 2. Changes in this pass

| File | Change / purpose |
|---|---|
| `AUDIT_REPORT.md` | Pre-edit audit and target architecture. |
| `include/fr10_pose_controller/cartesian_tracking.hpp` | Single pure calculation of position, along-track, perpendicular and orientation error, with nonfinite rejection. |
| `src/motion_adapter.cpp` | Replaced two copies of tracking-error arithmetic with that shared calculation. |
| `test/core/test_cartesian_tracking.cpp`, `CMakeLists.txt` | Deterministic geometry/nonfinite test and package registration; fixed relocated ABI-test workspace path. |
| `test/abi/test_fairino_v393_abi.py` | Scans `build_grouped`/`install_grouped` as well as legacy locations, so SDK inventory is checked against the actual workspace. |
| `tools/ode_vehicle_client.py`, `tools/run_ode_scenarios.py` | Isolated three-mode Gazebo test harness; wheel drive and odometry checks; per-case logs and summary. Optional temporary spawn, contact and solver experiments do not change production models. |
| `tools/ode_repro_client.py` | In mounted mode, finite joint test now evaluates j1–j6 rather than unrelated unregistered wheel-effort interfaces. |

Existing source edits in `DEBUG_ODE_REPORT.md` predate this pass. This report does not claim them as new changes.

## 3. Build and unit regression

Commands:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --build-base build_grouped --install-base install_grouped --symlink-install --packages-select fairino_hardware_v3_9_3 fr10_pose_controller vehicle_description vehicle_bringup swerve_vehicle_control --parallel-workers 2
source install_grouped/setup.bash
ctest --test-dir build_grouped/fr10_pose_controller --output-on-failure -j2
```

At the 2026-09-26 baseline, **5 packages finished** with no compiler errors; CTest had **0 failures out of 62**, one intentional skip (`extended_k_moveit_numerical_not_active`). The ABI test initially failed because its workspace path resolved to `src/`; after the path/inventory correction it passed and confirmed the adjacent SDK resolves from the hardware plugin. The newer build and 63-test result are in section 8.

## 4. Gazebo scenarios (fresh runs in requested order)

All runs use ROS_DOMAIN_ID=93 and Gazebo port 11393; commands below assume the two setup files above have been sourced. Each run writes `gazebo.log`, client logs, expanded URDF and `summary.json` under `docs/ode_debug/<label>/`.

| Scenario / command | Wall time | ODE LCP | Motion / state | Verdict |
|---|---:|---:|---|---|
| `python3 tools/run_ode_scenarios.py arm_postrefactor_20260926 arm` | 27.66 s | 0 | Action status 4; final max joint error 9.66e-6 rad; six joint fields finite | **PASS** for this trajectory |
| `python3 tools/run_ode_scenarios.py vehicle_baseline_20260926 vehicle` | 30.67 s | 1,125 | Wheel state finite; odometry moved 0.750 m | **FAIL**: repeated LCP |
| `python3 tools/run_ode_scenarios.py mounted_postrefactor_20260926 mounted` | 30.69 s | 12,466 | Arm action status 4, final max error 9.66e-6 rad; vehicle moved 0.736 m; original arm finite flag was polluted by NaN from unregistered wheel effort | **FAIL**: severe repeated LCP |

The `mounted` finite flag was corrected in the client after that baseline run. In the controlled mounted rerun below, the corrected six-arm-joint finite flag was true. The baseline `gazebo.log` contains no nonfinite tokens; the NaNs were only wheel effort state fields, not arm commands.

## 5. ODE diagnosis and temporary isolation results

The existing [DEBUG_ODE_REPORT.md](DEBUG_ODE_REPORT.md) already established that an old dynamic Dumpster and ground-touching fixed FR10 base collision caused standalone errors. With those existing fixes, the fresh arm-only run above has zero LCP. The vehicle-only test independently proves vehicle contacts are sufficient to trigger LCP. In the vehicle baseline, first LCP messages appeared after robot spawn/joint-state-broadcaster activation and before steering/drive controllers were activated. The mounted run also emitted LCP before the arm action. No `motion_adapter` process was launched. Six-arm joint action target and measured positions were finite; the driver/backend code rejects nonfinite joint commands.

The expanded mounted URDF has **31 inertial links**. All 31 masses and inertia tensors were finite, positive and positive definite under eigenvalue and principal-moment triangle checks. Vehicle examples: chassis 150 kg, minimum inertia eigenvalue 20.042 kg m²; each wheel 15 kg, minimum 0.177875 kg m²; each steering fork 3 kg, minimum 0.0065 kg m². The world uses ODE `world`, 1 ms step, CFM 1e-5 and ERP 0.2. Wheel contact stiffness is 1e6 for three wheels and 1e7 for wheel 4. Steering controller YAML lists p=300, d=10; this is not proof those values are the active Gazebo joint PID gains.

Temporary experiments, all on generated launch/model/world files, with production files unchanged:

| Vehicle or mounted experiment | LCP | Other result |
|---|---:|---|
| Vehicle, all wheel kp=1e6, original spawn z=0.1 m | 1,924 | Still fails; wheel-4 stiffness difference is not sufficient cause. |
| Vehicle, spawn z=0, original world | 168 | Smaller count, still repeated; vehicle moved 0.735 m. |
| Vehicle, spawn z=0, world CFM=0.001 | 36 | Still fails. |
| Vehicle, spawn z=0, world CFM=0.01 | 4 | Four startup errors in one short cluster after a station spawn; not proven repeatable. Vehicle moved 0.732 m. |
| Mounted, spawn z=0, world CFM=0.01 | 52, with 15 `-inf` solver values | Arm fields finite, action succeeded, vehicle moved 0.732 m; still fails. |
| Mounted, spawn z=0, world CFM=0.1 | 1,938, with 1,011 `-inf` solver values | Degraded; this is not a safe fix. |
| Vehicle, spawn z=0, wheel kp=1e5 | 6,909 | Degraded. |
| Vehicle, spawn z=0, CFM=0.01, ground max_contacts=2 | 4 | Wheel state finite, moved 0.746 m; still fails zero-error gate. |
| Mounted, same parameters, two runs | 4 then 19 (`-inf`: 2 then 7) | Arm action and wheel motion succeeded in both; repeated solver failures remain. |
| Vehicle, ODE `quick` solver 100 iterations | 0 before `gzserver` exit -11 | Invalid test: simulator crashed before controller activation. |

Prior controlled collision tests in `DEBUG_ODE_REPORT.md`: removing all arm collisions on the vehicle still left 263 LCP; removing wheel collisions caused an unsupported chassis to fall and 3,707 errors. Neither is a production fix. The current data points to vehicle ground/wheel/constraint interaction, but does not isolate a single offending collision pair or demonstrate a stable production parameter set. Solver warnings are not suppressed.

## 6. Acceptance checklist and next work

| Requirement | Status / evidence |
|---|---|
| Clear domain/adapter boundary | **Partial**: existing core/backend port is sound; `motion_adapter.cpp` still mixes policy/planning/ROS/diagnostics. |
| No duplicated logic | **Partial**: Cartesian error calculation consolidated; duplicate pose validation/policy logic remains. |
| `motion_adapter` only an adapter | **Not met**: it still plans and sequences insertion. |
| Pure domain unit tests | **Partial**: existing suite plus new tracking test pass; full extracted policy/planning tests remain. |
| Reuse across sim/physical | **Partial**: `RobotBackend` and ros2_control path exist; Gazebo scenario uses a separate controller graph. |
| Build and existing tests | **Met** for selected packages and runnable CTest suite. |
| FR10 standalone Gazebo | **Met** for tested home-to-target motion; zero LCP. |
| Vehicle standalone Gazebo | **Provisional pass**: current candidate has 0 LCP and measured 0.752 m motion in one full host-Gazebo run; repeat/Docker gates remain. Baseline had 1,125 LCP. |
| FR10 mounted on vehicle | **Provisional pass**: current candidate has 0 LCP and one coupled-motion run with 4.05 s arm/vehicle overlap; repeat/Docker gates remain. Baseline had 12,466 LCP. |
| Historical copy/paste count | **Unavailable**: the sole initial Git commit already contains the split package; no pre-split `fr10_pose` source is available. |

Next work is to capture exact contact pairs and joint forces during the first vehicle LCP, then vary one physical cause at a time: fork/chassis and wheel/ground geometry, friction/contact constraints and model spawn, with a vehicle-only and mounted gate after each change. The temporary CFM result does not justify changing production physics. Once physics is stable, extract pose policy/state, planning worker and diagnostics from `motion_adapter.cpp` into separate tested components. Repeat all three Gazebo scenarios and the full CTest suite before acceptance.

## 7. Continued vehicle contact investigation — 2026-09-27

Fresh passive vehicle with no active controllers produced 12,940 LCP; the first failure involved only four wheel/ground contacts. Contact trace showed rigid tangential wheel constraints. A controlled change to ODE collision slip1=slip2=1e-5 reduced passive V0 and rotating V7 to zero LCP while retaining the original friction direction and solver. Temporary tests with that slip also passed forward, lateral and diagonal motion with measured chassis displacement. The exact failing solver row remains unobserved.

The production candidate now lives in vehicle_bringup/src/wheel_contact_surface.cpp and loads only for vehicle models. Five-package build passed. Direct runtime observation found slip1=slip2=1e-5 on all four wheel collisions. Initial production V4 zero and V7 rotate runs each had zero LCP for about 28 seconds, finite state, and stable final pose. Full V0–V8 and three-repeat acceptance is still running; mounted has not been retested. See VEHICLE_LCP_ROOT_CAUSE.md and docs/ode_debug/vehicle_production_v4_surface_verified.

Expanded geometry independently confirms wheel center z=0.21 m and radius=0.21 m relative to base_footprint. Current vehicle spawn z=0.1 m drops the chassis 0.1 m; in the measured V4 run this caused up to 1.40 m/s downward speed, 0.094 m planar drift and 0.077 rad yaw before settling. A one-variable spawn-height test is pending. The architecture and workspace granularity gates remain unmet, so overall status stays incomplete.

## 8. Current evidence and remaining gates — 2026-09-27

The vehicle candidate applies ODE `slip1=slip2=1e-5` to the four wheel collision surfaces at runtime, preserving configured friction direction, coefficients and controller limits. Vehicle spawn height was independently corrected from 0.1 m to 0.0 m: expanded URDF geometry places wheel centers 0.21 m above `base_footprint` with radius 0.21 m. The former 0.1 m drop caused about 0.094 m planar drift and 0.077 rad yaw before settling. The slip targets persistent tangential-contact failure; the spawn correction removes avoidable initial impact. The exact failing ODE constraint row remains unobserved.

Current production-candidate runs use **host Gazebo Classic**, ROS domain 93 and `tools/run_ode_scenarios.py`; generated launch/world files and raw logs are in the linked result directories. They are **not Docker runs**: the existing `fairino-container` did not have this workspace mounted. The controller/Gazebo execution graphs remain separate.

| Scenario / raw summary | Duration | LCP | Measured outcome |
|---|---:|---:|---|
| [Arm standalone](docs/ode_debug/arm_production_spawn0_run1_20260927/summary.json) | 27.81 s | 0 | Action status 4, six arm fields finite, max final joint error 9.66e-6 rad. |
| [Vehicle standalone](docs/ode_debug/vehicle_ros_pose_probe_20260927/summary.json) | 31.61 s | 0 | Actual chassis motion 0.752 m, finite pose, max tilt 0.00050 rad, settled speed 3.23e-9 m/s. |
| [Mounted basic](docs/ode_debug/mounted_ros_pose_run1_20260927/summary.json) | 33.89 s | 0 | Arm action status 4 and chassis motion 0.755 m; arm fields/physics finite. |
| [Mounted coupled](docs/ode_debug/mounted_coupled_probe_20260927/summary.json) | 37.12 s | 0 | Arm goal at rest, then second goal overlapped moving vehicle for 4.05 s; actual chassis motion 1.207 m, final arm error 9.63e-6 rad. |

The V0–V8 diagnostic matrix ran all nine cases once, each about 28.6–29.0 s with zero LCP, finite ROS/physics state and active-controller coverage; [V0 result](docs/ode_debug/vehicle_final_matrix_V0_20260927/measured_result.json). Examples: [V5 forward](docs/ode_debug/vehicle_final_matrix_V5_20260927/measured_result.json) moved 0.752 m, [V6 lateral](docs/ode_debug/vehicle_final_matrix_V6_20260927/measured_result.json) moved 0.734 m, [V7 rotation](docs/ode_debug/vehicle_final_matrix_V7_20260927/measured_result.json) turned 0.747 rad and [V8 diagonal](docs/ode_debug/vehicle_final_matrix_V8_20260927/measured_result.json) moved about 0.496 m on each planar axis. Long lateral motion beyond the basic gate hit an angled world barrier: the first LCP followed barrier contact by about 0.51 s; removing only that barrier in a generated test world gave zero LCP over 43 s and 4.46 m actual lateral travel. The production world was not modified. Obstacle impact safety is a separate unsolved scenario; these results establish basic free-space motion only.

The 2026-09-27 build command was `colcon build --base-paths src --build-base build_grouped --install-base install_grouped --symlink-install --packages-select fairino_hardware_v3_9_3 fr10_pose_controller --parallel-workers 2`; both packages finished. `ctest --test-dir build_grouped/fr10_pose_controller --output-on-failure -j2` reported **0 failed out of 63**, one intentional skip. The added ROS-free `TrajectoryTraceValidator` checks every timed sample through the IK/collision provider, FK, finite Cartesian error and insertion-only strict trace limits; `motion_adapter.cpp` delegates this check. The callback also no longer serializes the pre-insert pose into a second ROS message for duplicate parsing. This is a bounded extraction, not full God-file remediation.

Remaining before full acceptance: three consecutive fresh primary-scenario repetitions and the requested full-matrix repetitions; Docker/Gazebo execution with this workspace available; extraction of pose sequencing, scene-aware planning and diagnostics from `motion_adapter.cpp`, with pure tests and final full build/CTest/Gazebo regression. No physical FAIRINO hardware actuation was performed. Gazebo still emits a controller-period warning (10 ms controller vs 1 ms physics) without coincident LCP in these runs. One vehicle run logged `robot_state_publisher` exit -2 during intentional SIGINT shutdown; it was not a runtime physics failure.

## 9. Repeated host-Gazebo and architecture continuation — 2026-09-27

The first post-spawn-order scenario repeats now have three consecutive **vehicle** PASS results ([1](docs/ode_debug/vehicle_external_repeat1_20260927/summary.json), [2](docs/ode_debug/vehicle_external_repeat2_20260927/summary.json), [3](docs/ode_debug/vehicle_external_repeat3_20260927/summary.json)) and three consecutive **mounted coupled** PASS results ([1](docs/ode_debug/mounted_external_repeat1_20260927/summary.json), [2](docs/ode_debug/mounted_external_repeat2_20260927/summary.json), [3](docs/ode_debug/mounted_external_repeat3_20260927/summary.json)). Every one had 0 LCP, finite arm/vehicle data and no runtime `gzserver` crash. Vehicle actual displacement was 0.705–0.752 m. Mounted action status was 4, maximum joint error about 9.63e-6 rad, arm/vehicle commanded-motion overlap 4.05–6.16 s and actual chassis displacement 0.785–1.207 m. These are host Gazebo tests, not Docker tests. They test one short free-space trajectory, not obstacle impact.

The in-server `libgazebo_ros_state` diagnostic plugin was removed from generated regression worlds and replaced by a read-only Gazebo Transport observer in a separate process. Its first implementation subscribed before the pose topic was advertised; one run had zero samples and a startup `gzserver` exit -11. Waiting for advertised topic availability and recreating the transport node on a no-message retry produced pose traces on subsequent runs. This temporal association does **not** prove the earlier startup crashes were caused only by the observer: an older crash dump points into Gazebo `RaySensor` / ODE multiray code, and the exact current crashing stack was not captured. The production launch now spawns three charging stations sequentially before the vehicle, with fail-closed exit handling; a two-case launch unit test checks the transition helper. A GDB attempt altered startup timing enough to make station spawn time out, so it did not establish a crash backtrace. No production LiDAR sensor or physics warning was disabled.

The first 45-second V0–V8 matrix using the external pose observer passed all nine cases with 0 LCP and 42–43 seconds of measured chassis pose per case ([V0](docs/ode_debug/vehicle_transport_v0_45s_20260927/measured_result.json), [V5](docs/ode_debug/vehicle_transport_matrix1_V5_20260927/measured_result.json), [V6](docs/ode_debug/vehicle_transport_matrix1_V6_20260927/measured_result.json), [V7](docs/ode_debug/vehicle_transport_matrix1_V7_20260927/measured_result.json), [V8](docs/ode_debug/vehicle_transport_matrix1_V8_20260927/measured_result.json)). The initial 30-second V0 had 0 LCP but only 16.8 seconds of post-spawn pose, correctly failed the unchanged 20-second gate, and was rerun at 45 seconds. Repeat matrix 2 passed V0–V4; its first V5 session was interrupted without a summary and is being rerun under a new label. Matrix repeat gates are **not yet complete**.

The FR10 adapter now delegates named joint order, degree/radian conversion and nonfinite rejection to a ROS-free `joint_target_decoder.hpp`; the mutable cached joint-name map and its method were removed. A red compile (missing header) preceded implementation, then direct pure test and package build passed. The most recent full scoped build finished five packages; the existing Gazebo CMake developer warning appeared, with no compiler error. CTest now reports **0 failures out of 64 FR10 tests** (one intentional skip) and **1/1 vehicle launch test**. This extraction removes one domain decision from `motion_adapter.cpp` but does **not** meet the requested full policy/planning/diagnostic decomposition or whole-workspace eight-axis audit. No SimMachine/physical hardware exercise or Docker Gazebo run has been performed.

## 10. Controller readiness, three-repeat regression and insertion state — 2026-09-27

A station spawn previously timed out when requested before Gazebo `/clock` advanced ([V5 failure](docs/ode_debug/vehicle_transport_matrix2_retry_V5_20260927/summary.json)). `vehicle_bringup` now waits for advancing `/clock` before spawning station 1, then starts stations 2 and 3 and the robot serially. After this change, one vehicle run still aborted in `gzserver` with `rclcpp::RCLError: failed to send response: cannot publish data` while controller manager started (raw log (local: `docs/ode_debug/final_clock_vehicle_1_20260927/gazebo.log`)); a mounted run did not crash but its client exhausted a 25 s action-server wait before all spawns/controllers finished ([raw result](docs/ode_debug/final_clock_mounted_1_20260927/summary.json)). Neither was an ODE LCP error. The precise middleware failure cause is **not established**.

`tools/run_ode_scenarios.py` now waits for each required spawner's actual `Configured and activated` event before launching command clients, recording `controllers_ready` in the summary. Three pure parser tests and Python syntax check passed. This prevents premature client calls in the harness; it is not proof that the ROS middleware cannot fail.

| Host-Gazebo scenario | Three fresh results | Physics/server | Measured outcome |
|---|---|---|---|
| FR10 standalone | [1](docs/ode_debug/final_ready_arm_1_20260927/summary.json), [2](docs/ode_debug/final_ready_arm_2_20260927/summary.json), [3](docs/ode_debug/final_ready_arm_3_20260927/summary.json): 3/3 PASS, 28.45–44.73 s | 0 LCP each, no crash | Action status 4, finite arm values, maximum final joint error 9.66e-6 rad. |
| Vehicle standalone | [1](docs/ode_debug/final_ready_vehicle_1_20260927/summary.json), [2](docs/ode_debug/final_ready_vehicle_2_20260927/summary.json), [3](docs/ode_debug/final_ready_vehicle_3_20260927/summary.json): 3/3 PASS, 29.52–42.04 s | 0 LCP each, no crash | Actual planar motion 0.675–0.752 m; finite measured pose. |
| FR10 mounted on vehicle | [1](docs/ode_debug/final_ready_mounted_1_20260927/summary.json), [2](docs/ode_debug/final_ready_mounted_2_20260927/summary.json), [3](docs/ode_debug/final_ready_mounted_3_20260927/summary.json): 3/3 PASS, 42.58–62.34 s | 0 LCP each, no crash | Action status 4, arm/drive overlap 4.05 s, actual vehicle motion 1.192–1.209 m. |

`insertion_sequence_state.hpp` now owns pending staging generation, retained insertion target and duplicate-target comparison in one ROS-free class. `motion_adapter.cpp` delegates cancellation, direct/staged insertion, retract and generation-qualified consume; the old three insertion state fields are gone. This is a tested ownership boundary, **not** a complete policy/planning/diagnostic decomposition. The three Gazebo scenarios use `joint_trajectory_controller` directly and do **not** launch `fr10_pose_controller_node`, so they do not runtime-test this adapter change.

After the extraction, five scoped packages built successfully. FR10 CTest: **0 failed of 67**, one intentional skip; vehicle_bringup CTest: **2/2 passed**. The 3×3 Gazebo runs preceded the final insertion-state edit, which does not affect their launch graph. Remaining gates: full per-file eight-axis audit, remaining `motion_adapter.cpp` decomposition and runtime exercise, repeated V0–V8 matrix, angled-barrier impact case, and Docker/Gazebo with the workspace available. The available Docker image lacks ROS/Gazebo and the workspace mount; host runs are not Docker evidence.

## 11. Actual adapter runtime on FakeSystem — 2026-09-27

The existing `test_fake_runtime.py` was pointed at the freshly built `install_grouped` overlay through a verified test-only `FR10_TEST_SETUP` selector. Its previous hard-coded `install/` overlay was stale. `use_fake_hardware:=true` expands to `mock_components/GenericSystem`, not the FAIRINO driver. Because production `RosControlBackend` correctly requires fresh `/fairino_driver/health`, the first FakeSystem attempt stayed unready without health; a fixture-only 16-field healthy heartbeat was added at 200 Hz in an isolated localhost ROS domain. The production health requirement was **not** weakened. The packet/setup fixture unit tests are registered in CTest.

[Current FakeSystem result](docs/ode_debug/fr10_fake_adapter_state_health_20260927/fake_runtime.json): startup PASS, joint goal PASS (0 rad final error), and one-shot pose/insertion PASS (1.53e-6 m position error, 5.82e-6 rad orientation error). The live node logged `FREE_SPACE -> PRE_INSERT -> INSERTION` and a generation-qualified transition, exercising the extracted insertion state. The complete smoke script nevertheless exits **FAIL** on its 20-target, 10 Hz pose-stream case: the final measured TCP remains about 0.250 m from the requested last socket pose at PRE_INSERT. Logs show a Jacobian-condition rejection and later time-parameterization rejection during repeated retargeting; the controller safely holds rather than sending an invalid trajectory. This is a real unresolved regression-test failure, not a PASS or an ODE issue.

An attempted baseline comparison through the older `install/` tree failed before startup because that tree lacks `fr10_pose_controller.launch.py` (log (local: `docs/ode_debug/fr10_fake_precurrent_comparison_20260927/runtime.log`)). Thus this run cannot establish whether 10 Hz pose-stream behavior regressed in the latest extraction. The current public pose configuration describes one final socket target and autonomous staging; the older smoke script expects rapidly changing targets on that same topic. Resolve this contract/test mismatch and prove safety before changing retargeting behavior. Do not relax collision, Jacobian, timing or stop checks merely to make the stream test pass.

Latest verification: five scoped packages built; FR10 CTest **67 total, 0 failed, 1 intentional skip**; vehicle_bringup CTest **2/2 passed**; the nine primary host-Gazebo summaries in §10 still stand. Overall acceptance remains **incomplete**.

## 12. Three measured V0–V8 repeats and strict insertion-axis extraction — 2026-09-27

The diagnostic command for each case was `python3 tools/run_vehicle_probe.py LABEL Vx --duration 45 --no-probe --omit-state-plugin`, followed by `python3 tools/analyze_vehicle_probe.py docs/ode_debug/LABEL`, with Humble and `install_grouped` sourced. Each launch used a fresh Gazebo server and the analyzer required actual model-state duration, finite physics/ROS data, the expected controller set, stationary settling or measured motion, and zero LCP. The shared `ode_scenario_readiness.py` mapping now gates clients on controller activation; its four parser/mapping tests passed.

| Measured matrix | Valid cases | Actual model-state duration | LCP | Representative evidence |
|---|---:|---:|---:|---|
| First 45 s repeat | 9/9 PASS | 42.64–43.02 s | 0 each | [V0](docs/ode_debug/vehicle_transport_v0_45s_20260927/measured_result.json), [V5](docs/ode_debug/vehicle_transport_matrix1_V5_20260927/measured_result.json), [V7](docs/ode_debug/vehicle_transport_matrix1_V7_20260927/measured_result.json) |
| Second 45 s repeat | 9/9 PASS | 39.48–46.33 s | 0 each | [V0](docs/ode_debug/vehicle_transport_matrix2_V0_20260927/measured_result.json), [V5](docs/ode_debug/vehicle_transport_matrix2_ready_V5_20260927/measured_result.json), [V7](docs/ode_debug/vehicle_transport_matrix2_ready_V7_20260927/measured_result.json) |
| Third 45 s repeat | 9/9 PASS after V0 rerun | 42.94–54.70 s | 0 each | [V0 rerun](docs/ode_debug/vehicle_transport_matrix3_ready_V0_20260927/measured_result.json), [V5](docs/ode_debug/vehicle_transport_matrix3_V5_20260927/measured_result.json), [V7](docs/ode_debug/vehicle_transport_matrix3_V7_20260927/measured_result.json) |

A read-only recomputation of all 27 selected summaries and measured results confirmed 27 PASS, 0 LCP, finite ROS/physics state and no server crash *in those selected runs*. The initial third-matrix V0 attempt was **FAIL**: `gzserver` exited -11 immediately after robot spawn, with no model-state samples (raw log (local: `docs/ode_debug/vehicle_transport_matrix3_V0_20260927/gazebo.log`)); its successful rerun does not establish crash-free startup reliability. The older angled-barrier impact failure also remains outside this free-space matrix.

`LinearInsertPlanner::validateAxis` now owns the strict insertion/retraction segment, measured perpendicular deviation and signed forward-projection decision in the ROS-free domain. `motion_adapter.cpp` supplies measured FK and logs the result; it no longer implements that geometry. A test first failed to compile without the new API, then passed for forward insertion, retraction, lateral deviation, reverse travel and nonfinite input. The existing limits, topic API, 125 Hz executor and hardware path were unchanged. This is another bounded extraction, **not** full remediation of the 2,495-line adapter.

Post-change build: `colcon build --base-paths src --build-base build_grouped --install-base install_grouped --symlink-install --packages-select fairino_hardware_v3_9_3 fr10_pose_controller vehicle_description vehicle_bringup swerve_vehicle_control --parallel-workers 2` finished all five packages. The only stderr was the existing GCC LTO serial-LTRANS warning. FR10 CTest: **67 total, 0 failed, 1 intentional skip**; vehicle_bringup CTest: **2/2 passed**; the four readiness-unit tests and `git diff --check` passed.

Two fresh `ros2_control` GenericSystem runs after the axis extraction remain **not fully passing**. [Run A](docs/ode_debug/fr10_fake_adapter_axis_20260927/fake_runtime.json): startup and joint PASS, then one-shot INSERTION entered TRACKING but a single `Robot feedback read failed` / `COMM_FAILURE` occurred before the 30 s pose timeout; final pose error was 0.0907 m. [Run B](docs/ode_debug/fr10_fake_adapter_axis_repeat_20260927/fake_runtime.json): startup, joint and one-shot INSERTION PASS (1.53e-6 m position error), then the 20-target 10 Hz stream failed after another `COMM_FAILURE`; the last requested pose was 0.004 m away. The controller stopped rather than issuing unsafe commands. The failing feedback source (test health heartbeat versus joint-state freshness/callback scheduling) is not yet isolated. The earlier [FakeSystem run](docs/ode_debug/fr10_fake_adapter_state_health_20260927/fake_runtime.json) had no communication dropout but independently failed the final streamed goal after stale-plan and timing rejections. Do not attribute either failure to the axis validator without further evidence or relax the 50 ms safety watchdog merely to pass the fixture.

Read-only Docker inspection found the running `fairino_simmachine:v3.9.8` container has no workspace mount and no `ros2` or `gzserver` executable; these are host-Gazebo results, **not** Docker/Gazebo acceptance. Remaining gates: full per-file eight-axis audit, complete policy/planning/diagnostic decomposition and stream recovery, simulator startup/obstacle-contact stability, and a Docker image/mount with ROS/Gazebo. No physical FAIRINO hardware was actuated.

## 13. Bounded retarget recovery and 125 Hz insertion check — 2026-09-27

Later evidence supersedes only the earlier 10 Hz FakeSystem stream failure, not the overall incomplete verdict. A fresh no-dropout run before the recovery edit reproduced final generation 22 rejection: scalar Ruckig timing failed while the old path was moving, leaving the robot at PRE_INSERT roughly 0.250 m from the requested socket (raw log (local: `docs/ode_debug/fr10_fake_timing_reason_20260927/runtime.log`)). The new timing diagnostic preserves the last concrete failure message; this case was `Ruckig failed to generate scalar path timing`. It was not an ODE error or a nonfinite command.

`PathTimeParameterizer` now returns `REQUIRES_CONTROLLED_STOP` when a moving boundary remains infeasible after the configured scaling iterations. The node retains that target, performs its existing jerk-limited controlled stop and replans at HOLDING; a separate `timing_stop_attempts` counter permits at most one such stop per target, independent of stale-seed replans. A stationary timing failure still rejects the target. Stale activation has a bounded three-live-reseed policy followed by one controlled-stop replan for POSE; a further stale result is rejected. No configured velocity, acceleration, jerk, reference-lead, collision or singularity limit was relaxed, and no failed trajectory is sent to the backend. ROS-free tests cover the retry bounds, moving/standing timing outcomes and diagnostic detail.

Runtime verification used `test_fake_runtime.py` with a test-only healthy heartbeat and `mock_components/GenericSystem`, localhost-only isolated ROS domains, not FAIRINO hardware. Three independent post-recovery runs ([1](docs/ode_debug/fr10_fake_split_retry_20260927/fake_runtime.json), [2](docs/ode_debug/fr10_fake_split_retry_repeat_20260927/fake_runtime.json), [3 after final internal log label](docs/ode_debug/fr10_fake_split_retry_final_20260927/fake_runtime.json)) each passed startup, joint, one-shot insertion and 20 changing pose targets at 10 Hz. Final target generation was 23, final TCP error 1.53e-6 m, outcome `GOAL_REACHED`/`HOLDING`, zero SDK/write errors, and clean shutdown. The logs show generation 22 controlled-stop/replan and subsequent strict INSERTION. These three passes do not prove immunity to the earlier occasional 50 ms feedback dropout; its source remains unisolated.

The measured PRE_INSERT-to-INSERTION boundary previously called `LinearInsertPlanner::plan` on the 125 Hz tick, allocating a dense waypoint vector. The tick now calls `checkAxis`: a bounded O(1) segment/axis check that does not materialize waypoints. `CartesianPath::linearSegmentCount` is the shared waypoint-capacity rule used by both the full worker-side path and the lightweight check; huge finite inputs are rejected before integer conversion. Tests went red before the new API, then passed parity for forward/reverse/lateral/nonfinite/capacity cases. A fresh post-change GenericSystem run also passed all four phases ([result](docs/ode_debug/fr10_fake_lightweight_axis_20260927/fake_runtime.json)); one transient intermediate generation was rejected for a joint-position limit, while the final generation reached the socket safely. The tick still performs bounded analytical FK and bookkeeping; the node is not adapter-only.

Final scoped command: `colcon build --base-paths src --build-base build_grouped --install-base install_grouped --symlink-install --packages-select fairino_hardware_v3_9_3 fr10_pose_controller vehicle_description vehicle_bringup swerve_vehicle_control --parallel-workers 2`. Five packages built with only the pre-existing GCC serial-LTRANS warning. FR10 CTest: 68 total, 0 failed, 1 intentional skip; vehicle_bringup CTest: 2/2; four readiness tests passed. The current five-package build/CTest and GenericSystem smoke follow the code edits. The previous 3x3 and 27-case host-Gazebo measurements remain valid for their separate launch graph, but have not been rerun in Docker or on physical hardware.

Remaining acceptance gaps: the ~2,500-line motion adapter still owns policy, scene-aware planning and diagnostics; whole-workspace eight-axis audit is unfinished; sporadic Gazebo startup exit -11 and angled-barrier impact LCP are unresolved; Docker lacks a ROS/Gazebo workspace; and the 50 ms FakeSystem feedback dropout needs isolation. Overall status remains **incomplete**.
## 14. Post-recovery Gazebo rerun — 2026-09-27

After the bounded retarget recovery and lightweight insertion check edits, the same host-Gazebo regression harness was run again in the required arm → vehicle → mounted order. Humble and `install_grouped/setup.bash` were sourced, with ROS_DOMAIN_ID=93 and Gazebo port 11393. Each invocation launched a fresh server and saved its command, expanded URDF, logs and summary under `docs/ode_debug/<label>/`.

| Scenario / exact command | Duration | Physics and control evidence | Result |
|---|---:|---|---|
| `python3 tools/run_ode_scenarios.py post_recovery_arm_20260927 arm` | 28.14 s | 0 LCP, no nonfinite log tokens or server crash; action status 4; finite arm state; maximum final joint error 9.66e-6 rad ([summary](docs/ode_debug/post_recovery_arm_20260927/summary.json)). | PASS |
| `python3 tools/run_ode_scenarios.py post_recovery_vehicle_20260927 vehicle` | 29.82 s | 0 LCP, finite wheel/pose data; odometry 0.750 m and actual planar displacement 0.753 m; no server crash ([summary](docs/ode_debug/post_recovery_vehicle_20260927/summary.json)). | PASS |
| `python3 tools/run_ode_scenarios.py post_recovery_mounted_20260927 mounted` | 39.49 s | 0 LCP, finite data; arm goal completed at rest and during 4.05 s of drive overlap; arm action status 4, joint error 9.63e-6 rad; actual vehicle displacement 1.207 m ([summary](docs/ode_debug/post_recovery_mounted_20260927/summary.json)). | PASS |

The only Gazebo update-rate warning reports a 10 ms controller update versus a 1 ms simulation step. The vehicle log also records two auxiliary processes exiting with code -2 during the harness's SIGINT shutdown; the measured controller/physics gate passed and `gzserver` did not crash. These results cover free-space host Gazebo, not obstacle impacts. The simulation launch uses `joint_trajectory_controller` rather than `fr10_pose_controller_node`; this rerun therefore checks the physical model/controller graph and does not independently validate the node recovery logic. GenericSystem results in §13 provide that separate node-level evidence. Docker and physical hardware remain untested, and the architectural acceptance criteria remain incomplete.
