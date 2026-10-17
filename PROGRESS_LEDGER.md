# FR10 + Vehicle progress ledger

Current phase: internal admittance / input-pose uncertainty commissioning, 2026-10-05. Public runtime is immutable Stage51 `.colcon/releases/rb_v3_humble_20261005T045747Z.5tB5jD`; full15-package build/testPASS, aggregate254/0errors/0fail/21reportedskips,83toolsPASS. Exact-prefix135=134PASS/1intentionalnumericalIKSKIP; fresh Gazebo replay running.
Current objective: bounded input-pose error → validated preinsert → single-owner six-axis compliance/search → independent mechanical HOLD → explicit measured-axis withdrawal.
Current work: Stage47 Gazebo yaw+2°/125us fullFAIL at VERIFY, backend publication/result17, no controlled-stop acknowledgment and no node-first confirmation. Peak penetration433.222um passes500um but stopped keyed373.980um fails250um; not seating acceptance. Encoder1ms-source publication took57ms-wall near contact, violating unchanged50ms freshness. Stage51 derives encoder manager rate from the actual validated physics step; P3D publishes each physics update and fixed512-sample BasePoseVelocity retains a full20ms window at20kHz. Stage50 startup gate uses the same16ms pose-derived SO(3) velocity as the node, rather than instantaneous P3D solver twist; historical seated31 qualifies, settling48 does not. New ring regression FAIL before fix/PASS after. Fresh isolated Stage51 Gazebo yaw+2°/125us full replay running in owned223/11989; mounted default replay follows. Stage48 legacy and yaw+2° MuJoCo fullPASS, combined X−2/Ry−2/right duplicate-FT requested fault cyclePASS; mounted48 FAIL safely before arm motion while chassis settling. Planner48 fresh24/24PASS and Docker48 readonlyPASS/activeFT0/no actuation. All safety gates/source deadlines and125Hz sole command owner remain unchanged. Foreign197 Gazebo/native Docker untouched. Full backend matrix remains NOT ACCEPTED.
Current evidence: `test_results/fr10_admittance_pose_error_20261003/WORK_LOG.md`. Stage43 build15PASS/aggregate252/0fail/21reportedskips;12 ROS-free insertion tests PASS after two new tests reproduced Stage42 root failures. Completed yaw/XY scans retain their accepted center without owning another scan's feasibility; quiet reversed samples in a shrinking coupled envelope remain bounded observations rather than ordered physical walls. Exact-prefix134=133PASS/1intentionalSKIP and57pytestPASS; four Stage43 full cycles PASS (yaw−2° keyed47.639/penetration20.717 µm; X+2 mmRy+2°54.361/18.124 µm), including two same-input root reruns: Y−2 mm/Rx+2° keyed34.659 µm/penetration15.634 µm; diagonal2 mm/2° from folded keyed95.235 µm/penetration18.918 µm, both full HOLD/retract/acknowledgedstop/node-first shutdown. Stage41 MuJoCo combined X+2 mm/Ry−2° and X−2 mm/Ry+2° full cycles PASS, keyed68.527/63.832 µm and penetration17.316/18.921 µm; measured-axis retract/acknowledged stop/node-first shutdown PASS. Stage42 Y−2 mm/Rx+2° and diagonal2 mm/2° from folded fullFAIL/TRACKING14, safely stopped/no auto retract; retained, not accepted from stopped fit alone. Stage42 CAD cache matches43,680 original queries byte-for-byte and independent Bullet reference; sweep21.10→1.59 ms, eight original/eight prepared actual ODE fixtures PASS. Stage42 Gazebo+2°250 µs fullFAIL/NO_PROGRESS21/penetration974.886 µm/keyed933.702 µm; acknowledged stop/node-first shutdown/noLCP. Stage43 Gazebo125 µs measurement interrupted, runtime DEPTH_LIMIT9/result10 recorded, owned node-first closed; Stage43 legacy zero-error fullFAIL/NO_PROGRESS21 (stopped fit58.910 µm/penetration11.553 µm), acknowledged stop/no auto retract/node-first shutdown. Stage44 deadband/completion compatibility fix12/12corePASS; full15-package build/testPASS, exact134=133PASS/1intentionalSKIP, Python57PASS; legacy physical rerun fullFAIL/NO_PROGRESS21 (keyed55.245/penetration11.760 µm), acknowledged stop/no auto retract/node-first shutdown. Stage45 private legacy rotational-gain candidate is testing before public profile promotion; Stage44 +2° yaw post-HOLD torque fault full requested cyclePASS keyed44.889/penetration19.506 µm, typed torque6/result7/acknowledged stop/no automatic retract; Stage45 candidate internal success0 recorded but measurement harness interrupted before full acceptance; source600/180s and all gates unchanged. Stage41 Gazebo500 µs fullFAIL/NO_PROGRESS21/penetration1.081644 mm/keyed865.174 µm, acknowledged controlled stop/node-first shutdown/noLCP; this integration step is rejected. Full uncertainty/backend matrix remains NOT ACCEPTED. Fresh Stage42 Docker readonly10s PASS/1250reads/1239fresh/zero actuator commands/Auto unchanged, ft_sensor_active=0 and raw/reference zero; real FT, electrical/latch confirmation and industrial certification remain NOT ACCEPTED. Planner57 source files match Stage35 recorded24/24PASS exactly; this is historical planning evidence, not a fresh contact replay.
Historical baseline: build81/validation84 CTest110=109PASS/1intentionalSKIP and27Python PASS, mounted Gazebo31/MuJoCo31/native83 full cycles PASS. Native85 PASS from1.174m-distant pose, left seated by user request. Those results apply only to their recorded revisions and native wrench was a CAD proxy, not a real FT sensor. Foreign runtimes remain untouched.

## Gazebo charging-port insertion smoke, 2026-10-01

- Added the simple public switch `use_force:=true` to the unified bringup. It is
  fail-closed unless `use_arm:=true use_vehicle:=false use_gazebo:=true`.
- The default force world is the canonical-home fixture containing the supplied
  detailed `meshes/cong_sac/meshes/cong_sac.stl` socket visual from
  `vehicle_description` and a deterministic compliant plate. The planning proxy
  is published once after MoveIt subscribes; repeated scene updates no longer
  invalidate a validated corridor. `force_fixture_profile:=socket` remains
  available for the standalone `[0.55, 0, 0.65]` fixture.
- End-to-end Gazebo goal PASS: `APPROACH → CONTACT_DETECTED → FORCE_INSERTION
  → VERIFY_INSERTION → SUCCESS → HOLD`; result `success=true`, final depth
  `0.0051102937 m`, conditioned axial force approximately 10 N, controlled stop
  completed. Raw plugin peak was 29.420916 N before conditioning/tare.
- Reproduction and exact action goal: `src/vehicle_bringup/README.md`.
  Artifact: `test_results/fr10_force_control_20261001T0918Z/REPORT.md`.
- Unit route test: 21/21 PASS with `ROS_LOG_DIR` redirected to writable `/tmp`.
  No real robot contact was run; native FAIRINO force remains disabled.

## Baseline, 2026-09-28

- Git HEAD: `3c1be9c` (`swerve: exit command normalizer cleanly on launch shutdown`). The working tree already contains tracked edits to FR10 core, `motion_adapter.cpp`, vehicle bringup and ODE tools, plus untracked reports, tests, source and raw evidence. The vendor reference submodule is modified. These changes predate this ledger and must not be reset or included in an unrelated commit.
- `FR10_VEHICLE_MASTER_TASK.md` and the `AGENTS.md` master-specification target do not exist. The accepted user plan and observed source/configuration are the scope authority.
- Existing host-Gazebo A/B/C results in `docs/ode_debug/final_ready_*_20260927/summary.json` and `post_recovery_*_20260927/summary.json` use `joint_trajectory_controller` and the earlier `steering_controller`/`drive_controller` graph. They are valid historical evidence for that graph, not current swerve-plugin acceptance.
- Existing 27 V0–V8 measurements are likewise historical. `docs/ode_debug/vehicle_transport_matrix3_V0_20260927/measured_result.json` records a failed startup with no model samples and is retained as a failure.
- Existing GenericSystem node evidence: `docs/ode_debug/fr10_fake_lightweight_axis_20260927/fake_runtime.json` records JOINT, one-shot POSE and 10 Hz stream reaching HOLDING. This does not substitute for Gazebo or FAIRINO SimMachine.
- The last `build_grouped/fr10_pose_controller/Testing/Temporary/LastTest.log` contains 68 passed CTests, including one intentional skip. This precedes the new work. The `LastTestsFailed.log` is stale and does not overturn the last complete log.
- `fairino-container` is `fairino_simmachine:v3.9.8` on `fairino-net` at `192.168.58.2`, with no workspace mount. `GET /index.html` redirects to `/login.html`; no Web process has been started.

## Current runtime map and audit decisions

| Boundary | Current owner / finding | Decision record |
|---|---|---|
| Vehicle command | `simulation.launch.py` includes deprecated `controllers.launch.py` and `drive.launch.py`; these start `swerve_controller` plugin and `/cmd_vel` to `/cmd_vel_nav` normalizer. The current runner still requires old `steering_controller` and `drive_controller`. | **DR-01 accepted:** keep the built plugin as the sole production vehicle actuator owner; adapt the launch and acceptance harness to its actual interface. Keep split controllers only for explicit diagnostic V0–V8 fixtures. Revert this selection if the plugin fails interface/state/timing gates or a live deployment consumer requires the old graph. |
| FR10 Gazebo | Current `vehicle_bringup` uses JTC, while `fr10_pose_controller_node` publishes a position array to `fairino10_controller`. The pose node is not part of historical A/C Gazebo runs. | **DR-02 candidate:** add an explicit pose-controller profile with a position bridge and no competing JTC; audit model, frame and health behavior before implementation. Reject if it requires changing safety limits or creates two joint owners. |
| FR10 domain | `MotionExecutor` has one writer; `motion_adapter.cpp` still owns policy, worker and diagnostics. Core library is ROS-free; physical plugin is sole SDK actuator owner. | **DR-03 candidate:** extract only cohesive ownership after method/state audit and regression. Revert an extraction if behavior or 125 Hz timing regresses. **DR-05 accepted:** worker-only stop-reason CAS rule preserves latched faults; pure test and A/C runtime checks passed. |
| Physics | Wheel ODE slip `1e-5` and spawn z=0 were tested independently for free-space runs. Barrier impact LCP and intermittent startup crash remain. | **DR-04 accepted baseline:** retain these values while diagnosing one physical cause at a time. Reject any physics change if a fresh vehicle/mounted gate fails or the effect cannot be isolated. |

The in-scope packages are `vehicle_description`, `vehicle_bringup`, `swerve_vehicle_control`, `fr10_arm_control`, `fairino10_v6_moveit2_config`, `fr10_pose_controller` and `fairino_hardware_v3_9_3`; perception and PLC packages are audited for dependency and ownership, without assuming that they share FR10 actuator control. Gazebo uses one `GazeboSystem`/controller manager for the selected model; FAIRINO uses the versioned hardware plugin. The active public topics include `/cmd_vel`, `/cmd_vel_nav`, `/odom`, `/joint_states`, `/joint_command`, `/charging_port/raw_pose`, `/fairino10_controller/commands` and `/controller_status`.

## Threshold provenance

| Metric | Value, clock and window | Provenance / status |
|---|---|---|
| A/B/C repetitions | Three consecutive valid runs, fresh gzserver each | User acceptance; ESTABLISHED |
| Physics integrity | Zero LCP; no nonfinite required state or gzserver crash, over full run | User acceptance and existing regression contract; ESTABLISHED |
| FR10 control/feedback | 125 Hz target, feedback freshness 0.050 s steady time | `controller.yaml`, node parameters and backend; ESTABLISHED |
| FR10 goal | Joint error ≤0.002 rad; TCP position ≤0.002 m, at settled goal | `controller.yaml`; ESTABLISHED for corresponding policy |
| Strict insertion | Orientation ≤1 degree and perpendicular ≤0.002 m | `controller.yaml` insertion policy; ESTABLISHED |
| Gazebo chassis trace | ≥100 samples, ≥20 s wall and sim, tilt <0.05 rad, final speed <0.03 m/s | `tools/evaluate_gazebo_pose.py`; ESTABLISHED as inherited free-space diagnostic gate, not obstacle certification |
| Mounted overlap | ≥2 s wall overlap | `tools/run_ode_scenarios.py`; ESTABLISHED for existing coupled test |
| SimMachine soak | ≥50,000 cycles and 123.5–126.5 reported loop Hz | `test_loop_soak_runtime.py`; ESTABLISHED for that test only |
| Odometry drift, jitter p99/max, callback latency, startup reliability sample count | No approved numerical limit | PROVISIONAL; measure and document a justified budget before acceptance |
| Gazebo observer continuity | Maximum 1.0 s wall gap between the 20 Hz sampled transport poses | Engineering validity rule set before A repeat runs: a gap this long misses at least 20 expected samples and makes the observer unsuitable to certify motion; ESTABLISHED for A observer validity, not a control-performance budget |

## Acceptance matrix and iteration

Latest iteration ID: P3-Model01. Earlier baseline P0-01: Hypothesis: historical A/B/C PASS describes the current production graph. Experiment: compare raw summary controller names against current launch/config and inspect `swerve_vehicle_control/CMakeLists.txt`. Evidence: `post_recovery_vehicle_20260927/summary.json` requires `steering_controller` and `drive_controller`; current launch starts `swerve_controller`. Result: REJECTED. The historical result is retained with its original scope.

Rejected hypotheses: historical JTC/split-controller Gazebo tests alone validate the new swerve plugin or `fr10_pose_controller_node`; they do not exercise either component.

Files modified in P0-01: `PROGRESS_LEDGER.md` only. Files modified but not yet fully regression-validated: all pre-existing FR10/vehicle/tool working-tree edits listed by `git status`, plus this ledger. Build/test status: not rerun after ledger; prior 68-test log is historical.

| Gate | Status | Evidence / reason |
|---|---|---|
| A. FR10 standalone host Gazebo | NOT RUN on required shared model | All previous A runs, including `p2_final_arm_20260928_{1,2,3}`, used the separate vendor Gazebo arm model. User now requires the same FR10 arm model as mounted C; old A evidence is historical only and cannot satisfy acceptance. |
| B. Vehicle standalone host Gazebo | PASS for three current-model forward free-space runs; startup reliability issue open | `docs/ode_debug/p2_prod_defer_forward_20260928_{1,2,3}` are three consecutive fresh Gazebo measured runs after production LiDAR `always_on=false`; each has valid `/scan`, independent chassis pose, 0 LCP/crash. Historical with-ray SIGSEGV and separate -6 abort remain unresolved reliability defects, so overall success is withheld. |
| C. FR10 mounted host Gazebo | PASS on pre-instrumentation code; current-build 3-run refresh required | Three consecutive fresh runs `docs/ode_debug/p3_mounted_pose_coupled_20260928_{3,4,5}/summary.json` passed before feedback-diagnostic edit. One current-build coupled run `docs/ode_debug/p2_feedback_mounted_20260928_1/summary.json` passed with 0 feedback drops. Earlier severe drift and broader gates remain open. |
| D. FR10 SimMachine CLI | FAIL, now externally blocked for rerun | Startup auto-sync originally passed; runs 2/3 reached insertion/retract but restore caused reference-lag stops. After authorized container restart, measured mode=1 Manual; hardware plugin refuses activation and requires operator Auto. See `test_results/d_reset_startup_20260928_fixed/runtime.log`. |
| E. FR10 Web UI | NOT RUN | URL redirects to login; program identity unknown. |

## Iteration P1-01, 2026-09-28

Hypothesis: using the current `swerve_controller` plugin config and matching primary harness restores production-controller readiness without worsening free-space physics. Experiment: first run `test_simulation_controller_contract.py` red (four missing-function errors), then add one-owner config composition, select `swerve_controllers.yaml`, use `swerve_controller.launch.py` as the sole vehicle launch, and update the primary vehicle client/runner. Build `vehicle_description`, `fr10_arm_control`, `swerve_vehicle_control`, `vehicle_bringup`; run vehicle CTest and one fresh B/C run. Evidence: `build_grouped/vehicle_bringup/Testing/Temporary/LastTest.log` (3/3), `docs/ode_debug/p1_swerve_vehicle_20260928_1/summary.json` (0 LCP, actual 0.638 m, no crash), and `docs/ode_debug/p1_swerve_mounted_20260928_1/summary.json` (0 LCP, 0.649 m, arm action succeeded and 7.51 s overlap, no crash). Result: ACCEPTED for the production graph correction and these diagnostic scenarios. This does not certify the pose controller or final repeated A/B/C gates.

Files modified in P1-01: `src/vehicle_bringup/launch/simulation.launch.py`, `src/vehicle_bringup/CMakeLists.txt`, `src/vehicle_bringup/test/test_simulation_controller_contract.py`, `tools/ode_vehicle_client.py`, `tools/run_ode_scenarios.py`, and this ledger. Files not fully regression-validated: all pre-existing working-tree edits and P1-01 changes against the full A/B/C/D/E matrix. The diagnostic V0–V8 harness still assumes the old split-controller graph and must be repaired before reuse.

Next action: audit the P3 arm command/health/model/frame contracts, add a failing test for selecting one arm controller profile, implement a Gazebo pose-controller profile without changing the default JTC path or FAIRINO health gate, then build and run one fresh A pose-node smoke test. Preserve the existing plugin B/C evidence and record every failure. In parallel, instrument loop timing and feedback age without adding blocking work to either control tick.

## Iteration P2-D01, 2026-09-28

Hypothesis: the SimMachine pose/retract test measures the configured insertion phases correctly. Experiment: run startup and the two-mode test against verified `fairino_simmachine:v3.9.8` at 192.168.58.2 with the grouped install; inspect runtime log and expected geometry. Evidence: `test_results/d_startup_20260928_1/{preflight,startup}.json` passes network, exclusive owner, TF and initial sync. First motion run `test_results/d_two_mode_20260928_1/two_mode_summary.json` reported retract false, but `runtime.log` shows a successful 250 mm retract; test expected a 200 mm pre-insert. Result: REJECTED test-side hypothesis; first motion verdict INVALID for the retract/speed assertions. Corrected `test_two_mode_runtime.py` to read `controller.insertion.pre_insert_distance_m=0.250` from the installed YAML and derive target minus tool-axis distance; local config/geometry check passed.

Second run `test_results/d_two_mode_20260928_2/two_mode_summary.json`: insertion and retract reached, zero SDK errors and clean shutdown; restore failed because `runtime.log` line 735 records `REFERENCE LAG CONTROLLED STOP`, joint 3, fresh feedback sequence 5526, sent-versus-actual lead 0.017461 rad against configured 0.017453 rad. Result: REJECTED for D motion acceptance; this is a genuine safety stop, with no safety limit changed. The TCP speed assertion is also not trustworthy yet: observer uses callback arrival intervals and had 9.75 ms intervals amidst a nominal 100 ms stream, inflating measured speed to 0.157 m/s. Source message timestamps must be used or jitter accounted for before speed verdict. The original failed run is retained.

FR10 build: `fairino_hardware_v3_9_3` and `fr10_pose_controller` passed. CTest: 1 unexpected failure of 68 (`extended_k_analytical_benchmark`), one intentional skip. The isolated repeat also failed: p99 88.89 us, over the source-defined 85 us limit in `test_kinematics_extended.cpp`; this is a real test failure, not waived as noise. Remaining tests passed. Files modified in this iteration: `src/fr10_arm_control/fr10_pose_controller/test/simmachine/test_two_mode_runtime.py` and this ledger. No physical hardware moved.

Next: instrument source-stamped TCP velocity measurement, compare callback delay, and isolate why the 125 Hz SimMachine restore generated a fresh reference lead just above 1 degree. Keep the configured safety lead and feedback timeout. For P3, add an explicit Gazebo position-controller profile using the vendor model for A, with the physical health gate unchanged; then validate the profile's single joint owner and run an A smoke before touching the mounted model.

## Iteration P3-A01, 2026-09-28

Hypothesis: the existing vendor FR10 model can run `fr10_pose_controller_node` in Gazebo when the position-array bridge is the only j1–j6 command owner and simulation does not wait for FAIRINO health packets. Audit: vendor model has `base_link`, `flange_link`, `tool0`; vendor Gazebo launch hardwired a JTC controller YAML while the pose backend writes `Float64MultiArray` to `fairino10_controller/commands`. Selected a profile parameter that preserves vendor JTC default; `use_gazebo=true` passes the pose package's existing `ros2_controllers.yaml`, disables unused move_group, uses sim time and explicitly disables hardware health gating only for that profile. FAIRINO launch default remains health-gated. Source files: `fairino10_v6_robot.urdf.xacro`, vendor `gazebo.launch.py`, pose launch, `node_parameters.hpp/.cpp`, `motion_adapter.cpp`. A fresh build of `fairino10_v6_moveit2_config` and `fr10_pose_controller` passed. Expanded Xacro/YAML check confirmed one position controller and required frames. Fresh `docs/ode_debug/p3_pose_arm_smoke_20260928_1/summary.json` reached a joint target; fresh `.../p3_pose_arm_smoke_20260928_2/summary.json` reached both joint and Cartesian pose, with active `position_controllers/JointGroupPositionController`, zero LCP and no gzserver crash. Result: ACCEPTED as a one-run A pose-node smoke, not A acceptance. The current smoke records controller/joint/TCP evidence but must add independent Gazebo link truth, /clock and timing trace before final three-run classification. `tools/run_fr10_pose_gazebo_smoke.py` was added.

Mounted-model audit: `vehicle_description` uses `fr10_base_link` and a wrist plate rotated by pi, with `charging_tcp`; it has neither vendor `base_link` as arm root nor `tool0`/`flange_link`. Replacing it with the vendor wrist would change the established mechanical convention and is out of scope without approval. DR-02 remains open for C: supply a model-aware base/TCP and matching MoveIt SRDF/IK geometry without altering the mechanical model, or reject a candidate that cannot preserve this contract. A vendor-model A PASS cannot substitute for C.

Next: add source-clocked, independent Gazebo joint/link/clock and tick-jitter evidence to A harness, then inspect C's arm-root/TCP/MoveIt dependency path and choose a model-aware adapter only after proving matching FK for custom URDF. Separately diagnose D reference lag on the restore path and the benchmark p99 failure without raising safety/timing limits. Final three-run A/B/C and E remain pending.

## Iteration P2-D02, 2026-09-28

Hypothesis: the reported 0.157 m/s insertion peak is physical. Experiment: record both source stamp and callback arrival for each TCP pose; recompute phase velocity from source stamps, preserving the original CSV and run artifacts. Evidence: `test_results/d_two_mode_20260928_3/{two_mode_summary.json,two_mode_tcp.csv,runtime.log}`. Source stamps are monotonic, insertion peak is 0.01668 m/s (the previous callback-time peak was an observer artifact), insertion and retract reached, SDK errors zero. Restore again triggered a fresh-feedback reference-lag controlled stop at joint 3: 0.018126 rad vs unchanged 0.017453 rad safety lead. Result: ACCEPTED measurement correction, REJECTED D motion acceptance. Files modified: `test/simmachine/ros_health.py`, `test_two_mode_runtime.py`, and ledger.

One-variable diagnostic P2-D03: temporary `test_results/d_velocity_trial_20260928/overlay.yaml` set only FREE_SPACE joint velocity scale to 0.70. `run1` inserted and retracted, then restore was rejected with `IK_FAILURE` at direct waypoint 0; no reference-lag stop. The simulator's starting joint state had been left displaced by prior failed restores, so the trial is **INVALID for testing the velocity hypothesis**. It does demonstrate that the restore target was not reachable from that starting configuration under current route selection. No production speed or safety setting was changed. Next: restart the verified SimMachine container to a known initial pose, record its measured joints, and repeat baseline/overlay from matching initial states before interpreting speed. Do not convert the missing reference-lag event in this invalid trial into a PASS.

## Iteration P2-D04, 2026-09-28

The verified `fairino-container` was restarted to recover a comparable initial simulated state. The immediate preflight `test_results/d_reset_startup_20260928/{preflight,startup}.json` was INVALID because ports 8080/20005 were still starting. An optional overlay argument regression in `test/simmachine/common.py` caused a malformed blank `controller_overlay:=` in the next startup attempt; repaired it to append the argument only when nonempty. The subsequent `test_results/d_reset_startup_20260928_fixed/runtime.log` shows SDK connection established and measured robot `enable=1 mode=1 ... error=0/0`, followed by the FAIRINO plugin's explicit `Robot is not in Auto mode. Operator must select Auto before hardware activation; Mode(0) is not called automatically.` The ROS controller manager then aborted during activation. Verdict: INVALID startup for D acceptance, concrete external state blocker for further D motion runs. No automatic mode/enable/reset API was called, no real hardware was actuated, and the safety gate is unchanged. E remains login-gated and no process was triggered.

Next for D: an authorized operator must place only the verified SimMachine instance into Auto mode through its normal interface, then rerun `test_ros_startup.py` with `FR10_TEST_SETUP=/home/tin/Vinuni/install_grouped/setup.bash`; record measured initial joints and effective mode/health. Only after readiness passes, compare baseline and single-variable velocity overlay from matching initial states. Do not alter the FAIRINO plugin's Auto preflight or hardware safety configuration. Continue independent A/B/C and architecture/timing work meanwhile.

## Iteration P3-A02, 2026-09-28

The first independent A truth attempt `docs/ode_debug/p3_pose_arm_truth_20260928_1/summary.json` is INVALID for final A because vendor Gazebo did not publish `/gazebo/link_states`; it still reached joint/pose goals with no LCP, and `/clock` advanced 23 s. The read-only Gazebo transport observer was generalized with default-preserving `ODE_WORLD_NAME`/`ODE_TARGET_ENTITY`; on the next fresh run `docs/ode_debug/p3_pose_arm_transport_20260928_1/summary.json`, the observer recorded 619 `fr10::wrist3_link` samples over 21.916 s simulation and 0.03171 m actual link displacement, while the pose node reached both goals with active position controller, zero LCP, no server crash, and clean observer exit. Timing snapshot: loop 125.012 Hz; period p50/p95/p99 7.75/8.25/8.75 ms (0.5 ms histogram bins), execution p99 0.75 ms, zero tick execution deadline misses, max feedback age 21.805 ms, Gazebo real-time factor 0.625. These are measurements, not proof against an unproven p99 budget. Before A repeat runs, the harness now requires observer alive through measurement, clean exit, finite/monotonic samples and <1.0 s wall gap; rationale and scope are in the threshold table. The raw first valid transport trace meets that rule (max gap 0.0684 s). One valid A run on this candidate; two consecutive repeats remain.

Files modified in P3-A02: `motion_adapter.cpp` (fixed-size per-tick histograms; percentiles computed in diagnostics), `tools/ode_transport_observer.cpp`, `tools/run_fr10_pose_gazebo_smoke.py`, ledger. FR10 build passed after instrumentation. The pre-instrumentation A run had mean loop period 8.0008 ms; the matched instrumented run had 7.9988 ms. This comparison does not establish a worst-case overhead budget. CTest after this change remains to run; earlier benchmark failure remains open.

## Iteration P3-A03, 2026-09-28

Three consecutive fresh host-Gazebo arm runs of the same position-controller profile passed the full measured smoke contract: `docs/ode_debug/p3_pose_arm_transport_20260928_2/summary.json`, `_3/summary.json`, `_4/summary.json`. Each reached a joint command and Cartesian pose command through `fr10_pose_controller_node`; controller manager listed only the active `position_controllers/JointGroupPositionController` on j1–j6. Independent Gazebo transport wrist samples: 637/636/638; actual displacement 0.03262/0.03241/0.03257 m; simulation clock advanced 22.9/22.4/23.1 s; maximum observer wall gap 0.068 s. All had zero ODE LCP, no gzserver crash, observer alive through measurement and clean exit. Period p99 8.75 ms on all three; execution p99 0.75/0.75/1.25 ms; zero tick execution deadline misses; max feedback age 16.831/32.277/20.653 ms, below the configured 50 ms validity gate. This establishes A for the defined joint+pose scenario only. Broader arm case catalogue, workload-specific realtime budget, and C remain outstanding. Previous `...truth_1` attempt is retained INVALID because the vendor world lacks ROS `/gazebo/link_states`.

Next: run three fresh B vehicle-plugin scenarios with actual Gazebo pose and active controller evidence; instrument/report vehicle command-to-state timing and odometry/TF against ground truth. Then tackle mounted custom-model base/TCP/MoveIt contract without altering mechanical convention. A must be rerun after any code/config/model change affecting its graph.

## Iteration P3-C01 and P2-VehicleLongHold, 2026-09-28

C model-aware profile is a candidate, not accepted architecture. `vehicle_bringup` now has an opt-in `arm_controller_profile:=pose` (default JTC unchanged), a custom SRDF group from `fr10_base_link` to `charging_tcp`, Gazebo-only health override and one position controller; the swerve plugin remains the sole wheel owner. MoveIt FK/IK/workspace checks were changed to arm-base-relative, and the analytical wrist-to-TCP fixed transform is derived from the unchanged URDF only when requested by this profile. Mounted startup `docs/ode_debug/p3_mounted_pose_startup_20260928_1/runtime.log` loaded the model, passed analytical/MoveIt FK cross-check, activated position + swerve controllers and reached READY. Build of FR10 and bringup passed; controller composition test 5/5 passed. No joint limit or mechanical convention changed.

Coupled motion `docs/ode_debug/p3_mounted_pose_coupled_20260928_1/summary.json` is FAIL: vehicle moved 0.463 m, arm pose target was rejected at waypoint 0, arm remained still, 0 LCP. Planning-worker diagnostic build and fresh run `..._2/summary.json` established the cause: `runtime.log` reports `seed_valid=false seed_reason=PlanningScene self/world collision detected endpoint_count=0 endpoint_reason=PlanningScene self/world collision detected`. This is evidence of collision validation at the start state, not proof that the goal or model can be accepted. Do not disable collision checking wholesale. Next isolate exact contact pairs with MoveIt collision contacts and classify intentional adjoining model contacts versus true chassis/tool interference before an ACM change.

A second critical observation in C run 2: independent Gazebo chassis transport trace rose from 0.406 m at sim t=13.9 s to 2.136 m at t=47.7 s after the five-second /cmd_vel command ended, with apparent translational speed increasing; no LCP was logged. The first run drifted only to 0.463 m. B short forward runs 1/2/3 each had actual 0.636/0.647/0.644 m, finite wheel/odom, zero LCP, no crash; the swerve plugin watchdog warnings occur after command silence and do not alone prove stability. Result: B ACCEPTED only for the defined short forward scenario; long-hold/mounted stability is unresolved high severity. Next record command timing, wheel velocity, odom and Gazebo pose through the post-stop window; compare mounted JTC 100 Hz and mounted pose 125 Hz one variable at a time. Reject a physics/control change if the same measured long-hold gate still creeps or if it destabilizes B.

Files modified in this phase: `moveit_ik_provider.hpp/.cpp`, `node_parameters.hpp/.cpp`, `motion_adapter.cpp`, `vehicle_bringup/launch/simulation.launch.py`, `package.xml`, `config/fr10_vehicle_pose.{srdf,yaml}`, `test/test_simulation_controller_contract.py`, `tools/run_mounted_pose_gazebo_smoke.py`, ledger. These have not passed final C or A after the latest MoveIt provider edit; A needs rerun after final relevant code changes. Latest iteration ID P3-C01. Current objective: isolate collision pairs and post-stop vehicle drift before any more C acceptance attempts.

## Iteration P3-C02, 2026-09-28

Hypothesis: the mounted profile's empty SRDF collision matrix causes MoveIt to reject the home state on intentional adjacent/fixed assembly overlaps. Experiment: launch the pose profile with no vehicle drive and request a 20 mm arm-root-relative pose; instrument the planning worker to report seed/endpoint validity and up to 12 collision pairs/depths. Evidence: `docs/ode_debug/p3_mounted_collision_pairs_20260928_1/{runtime.log,summary.json}`. The planner rejected waypoint 0. The first 12 contacts were camera_rear_mount–camera_support_right (0), camera_support_left–wrist_mount_backplate (0), charger_cradle_left–charger_rear_mount (3 mm), charger_cradle_left/right–charging_connector_link (4.482 mm), charger_rear_mount–charging_connector_link (8.5 mm), charger_support_left/right–wrist_mount_backplate (0), forearm_link–upperarm_link (0.836 mm), fr10_base_link–shoulder_link (3.8 mm), shoulder_link–upperarm_link (1.7 mm), and wrist3_link–wrist_mount_backplate (1.5 mm). The cap may hide further contacts. Gazebo had zero LCP and only ~3 mm chassis displacement without drive. Result: ACCEPTED root-cause evidence for start-state collision rejection; C remains FAIL. No SRDF exclusion or physical model was changed in this diagnostic.

Next: add only these demonstrated adjacent/fixed assembly pairs to `src/vehicle_bringup/config/fr10_vehicle_pose.srdf`, build `vehicle_bringup`, rerun the no-drive diagnostic with fresh gzserver and inspect any newly exposed contacts. Do not disable all collision checking or mark C PASS from diagnostic mode. In parallel, measure command, wheel and independent Gazebo pose over a long post-stop hold before changing vehicle control or physics.

## Iteration P3-C03, 2026-09-28

The first post-SRDF diagnostic `docs/ode_debug/p3_mounted_collision_pairs_20260928_2/summary.json` is INVALID for collision acceptance: pose node reported one fresh feedback sample aged 51.753 ms (>50 ms configured timeout) and stopped with COMM_FAILURE before the pose command was processed. Independent observers and gzserver survived; this is an intermittent feedback freshness defect, not permission to extend the safety timeout. A fresh repeat `..._3/{runtime.log,summary.json}` processed the command and exposed four additional contacts after the first 12 exact pairs were excluded: charger_cradle_right–charger_rear_mount (3 mm), charger_rear_mount–charger_support_right (0), charger_support_left/right–charging_connector_link (2.5 mm). The links are fixed to `plate_frame` in `fr10_wrist.xacro`. Result: ACCEPTED that the exact exclusions reveal only further rigid assembly contacts so far; REJECTED that the first 12 alone clear the seed. Four exact pairs were added to the mounted SRDF; collision checking for all unlisted pairs remains active. C remains FAIL.

Next: build `vehicle_bringup` and rerun no-drive collision diagnostic. If the seed is valid, measure the arm motion through the same profile before combining with vehicle drive. Separately record source and receive timestamps on joint states to isolate the single-sample 50 ms dropout and preserve its log.

## Iteration P3-C04, 2026-09-28

After 16 exact exclusions, the fresh no-drive run `docs/ode_debug/p3_mounted_collision_pairs_20260928_4/{runtime.log,summary.json,joint_timing.json}` reached READY and processed the pose command. MoveIt still rejected the seed and exposed only two additional fixed camera-fixture contacts: camera_rear_mount–camera_support_left and camera_support_right–wrist_mount_backplate, both logged at zero contact depth. These two are fixed to `plate_frame`; they were added to the mounted SRDF. Result: ACCEPTED that the collision matrix is narrowing to intentional rigid assembly contacts; REJECTED that the first 16 exclusions suffice. C remains FAIL. In this run, the separate Python observer recorded a 299 ms receive gap while ROS joint-state source stamps advanced in 8 ms steps; the pose controller itself reported max feedback age 24.482 ms and stayed READY. Thus the 299 ms gap alone proves observer callback scheduling delay, not a publisher hiatus or controller failure. The prior 51.753 ms controller freshness failure still needs direct timestamp tracing.

Next: rebuild `vehicle_bringup`, rerun the no-drive mounted diagnostic, and inspect every remaining pair. Do not infer a clean collision state until the route no longer fails. For feedback, measure source/receive time in the controller's backend around a failure; do not raise its 50 ms threshold.

## Iteration P3-C05, 2026-09-28

Fresh mounted no-drive diagnostic `docs/ode_debug/p3_mounted_collision_pairs_20260928_5/{runtime.log,summary.json}` reached READY, processed the pose command, had zero LCP and no gzserver crash. MoveIt now reports `seed_valid=true`; endpoint IK found two candidates. The free-space joint route still failed at waypoint 0 with a generic message, so the next hypothesis is path validation or endpoint Cartesian mismatch, not start-state collision. Result: ACCEPTED that the exact 18 SRDF exclusions clear the home collision; REJECTED that they alone make the pose route executable. `cartesian_route_planner.cpp` now preserves the failing interpolation sample and provider reason for diagnosis, without changing candidate selection, collision policy or limits. This edit requires build/unit/runtime regression.

Next: build `fr10_pose_controller`, run its Cartesian route planner tests, and rerun the same no-drive mounted scenario with fresh gzserver. Use the reported sample/provider reason to select the smallest correction. Then run long post-stop vehicle drive with wheel state, odom and Gazebo truth; no C acceptance yet.

## Iteration P3-C06, 2026-09-28

Fresh run `docs/ode_debug/p3_mounted_collision_pairs_20260928_6/{runtime.log,summary.json}` with the unchanged 18-pair SRDF and route failure instrumentation: seed valid, two endpoint IK solutions, first interpolated joint sample rejected by PlanningScene self/world collision. Zero LCP, no gzserver crash, no arm motion. Result: ACCEPTED that an intermediate state is colliding; the exact pair is still unobserved, so no further collision exclusion is justified. The planning-worker `MoveItIKProvider::validateState` rejection path now requests one contact from its immutable scene snapshot and includes pair/depth in the error; the control tick and collision decision are unchanged. Rebuild and test required.

Next: rebuild FR10, rerun `cartesian_path` and fresh mounted no-drive diagnostic; inspect first interpolated collision pair. If it is an arm/vehicle or tool/vehicle pair, do not add an ACM exclusion without physical geometry analysis.

## Iteration P3-C07, 2026-09-28

The fresh no-drive mounted attempt `docs/ode_debug/p3_mounted_collision_pairs_20260928_7/runtime.log` was interrupted during Gazebo startup when the agent turn was aborted. It has no summary, no measured pose/clock gate and no route result; no gzserver or observer process remains. Result: INVALID for the route hypothesis. The earlier FR10 build and `cartesian_path` CTest passed; the contact-pair diagnostic code remains built.

Next: rerun the exact no-drive mounted diagnostic under a new label and fresh gzserver; classify its first intermediate collision pair. Keep the interrupted artifact and do not count it toward A/B/C repetitions.

## Iteration P3-C08, 2026-09-28

Fresh mounted no-drive run `docs/ode_debug/p3_mounted_collision_pairs_20260928_8/{runtime.log,summary.json}` was valid for route diagnosis: READY, independent Gazebo pose and clock, zero LCP, no crash. Seed valid; endpoint IK found one candidate. Joint-route interpolation failed at sample 5 because `charger_rear_mount` contacted `charger_support_left` at logged depth 0.000000 m. Both are fixed to `plate_frame` in `fr10_wrist.xacro`; that one exact pair was added to the mounted SRDF. Result: ACCEPTED route contact hypothesis, but C still FAIL because arm did not move. The previous interrupted run 7 remains INVALID.

Next: rebuild `vehicle_bringup` and rerun no-drive mounted route. Continue exact pair classification until the route is valid or a true arm/vehicle contact appears. Independently run a drive-only long-hold mounted diagnostic with the newly added normalized-command, wheel-state, odom and Gazebo transport trace to diagnose post-stop drift.

## Iteration P3-C09, 2026-09-28

Fresh no-drive mounted run `docs/ode_debug/p3_mounted_collision_pairs_20260928_9/{runtime.log,summary.json}` proved the measured fixed-contact exclusion allowed the free-space route: MoveIt reached pre-insert and queued insertion; independent Gazebo wrist displaced 0.2361 m, no LCP/crash, joint feedback source gaps remained 8 ms and controller max feedback age was 8.283 ms. The diagnostic harness waited only 15 wall seconds, then issued SIGINT while insertion was still TRACKING. Subsequent backend write failure/COMM_FAILURE happened during intentional shutdown and is not evidence of spontaneous runtime dropout. Result: ACCEPTED for route and measured arm movement; INVALID for final insertion/settling because the observer stopped too early. The diagnostic wait was extended to 65 s before the next run. C remains unaccepted.

Next: rerun no-drive mounted insertion through measured completion; require final TCP and HOLDING status plus independent wrist trace, physics and feedback validity. Then run drive-only long-hold to isolate vehicle drift before a coupled C attempt.

## Iteration P3-C10, 2026-09-28

Fresh no-drive mounted run `docs/ode_debug/p3_mounted_pose_stationary_20260928_1/{summary.json,runtime.log,joint_timing.json,vehicle/transport_state.jsonl,wrist/transport_state.jsonl}` completed the original 20 mm arm-root-relative insertion target through the actual pose node and position bridge. Final status `HOLDING outcome=GOAL_REACHED pose_policy=INSERTION`; measured TCP position error 9.98e-6 m; independent Gazebo wrist displacement 0.019997 m; chassis displacement 0.003040 m; 720/718 vehicle/wrist transport samples, >39 s simulation, zero LCP/crash, both observers alive and cleanly exited. Joint source gap max 8 ms; independent observer receive gap max 38.5 ms; pose node max feedback age 8.2 ms; tick execution misses zero. Result: ACCEPTED for mounted arm insertion with vehicle stationary. This is not C acceptance because simultaneous vehicle motion and post-stop stability remain unverified. The harness's generic `pass=false` is expected in `--diagnostic` mode because it requires nonzero drive overlap; the specific no-drive goal evidence is valid.

Next: run `tools/run_mounted_pose_gazebo_smoke.py docs/ode_debug/p3_mounted_drive_only_20260928_1 --domain 95 --port 11395 --drive-only` with fresh gzserver. Align `drive_stop_wall_s` with `vehicle_ros_trace.json` normalized commands, wheel velocities, odom and independent transport pose. Decide whether drift is commanded, driven by wheel state, or physics before changing control.

## Iteration P2-V01 / P3-C11, 2026-09-28

Fresh mounted drive-only long-hold diagnostic `docs/ode_debug/p3_mounted_drive_only_20260928_1/{summary.json,vehicle_ros_trace.json,vehicle/transport_state.jsonl,wrist/transport_state.jsonl}` used the pose-profile controller graph with arm in HOLD and no planning. `/cmd_vel_nav` carried 100 positive 0.12 m/s commands over 5.03 wall s, then exactly one zero and no later nonzero command. Independent Gazebo chassis moved to x=0.6052 m at stop and x=0.6171 m after 45 wall s, so post-stop displacement was 0.01191 m. All four measured wheel velocities settled near 0.0009805 rad/s, consistent with odom vx≈0.000206 m/s and the continuing ~0.00022 m/s Gazebo translation. Final 1 s max actual speed 0.000225 m/s and max tilt 0.000548 rad meet the inherited free-space stability thresholds; zero LCP/crash. Result: ACCEPTED as diagnostic evidence that a small persistent wheel-speed bias remains after a zero command, but the earlier 2.16 m mounted drift did not reproduce without arm motion/planning. The generic harness `pass=false` is expected for `--drive-only` because no arm target was issued; this is not C acceptance. Do not erase the previous severe run.

Next: run one fresh coupled insertion plus 5 s drive with the same normalized-command, wheel-state, odom and Gazebo truth trace. Align `drive_stop_wall_s` with body and wheel dynamics, require final speed <0.03 m/s and tilt <0.05 rad per `evaluate_gazebo_pose.py`; compare against the drive-only control. If large drift recurs, isolate whether it correlates with arm insertion or command path before changing physics/controller.

## Iteration P3-C12, 2026-09-28

First fresh full coupled run on the current mounted pose/SRDF candidate: `docs/ode_debug/p3_mounted_pose_coupled_20260928_3/{summary.json,runtime.log,vehicle_ros_trace.json,vehicle/transport_state.jsonl,wrist/transport_state.jsonl}`. The pose node completed insertion (`HOLDING/GOAL_REACHED`) with final TCP error 1.07e-5 m. TCP moved 0.21675 m during 5.03 s vehicle drive (51 measured samples); independent Gazebo chassis moved 0.60818 m. Post-stop actual displacement 0.00345 m, final 1 s max speed 0.000231 m/s, max tilt 0.000565 rad, no normalized nonzero command after stop, zero LCP/crash; both observers alive and cleanly exited. Pose-node 125.010 Hz, period p99 8.25 ms, execution p99 0.25 ms, zero tick deadline misses, max feedback age 8.21 ms. Result: ACCEPTED as coupled run 1/3 for this defined scenario. The previous run with >2 m drift remains a historical defect and requires an independent reliability analysis; one clean run does not erase it.

Next: run two more consecutive fresh coupled runs without code/config changes under labels `p3_mounted_pose_coupled_20260928_4` and `_5`; require the same complete summary gates, final HOLDING/GOAL_REACHED and no abnormal post-stop drift. Any FAIL/INVALID resets the C run sequence. Then rerun A because MoveIt provider code changed after its prior three-run set.

## Iteration P3-C13, 2026-09-28

Three consecutive fresh coupled mounted runs on the unchanged pose/SRDF candidate, `docs/ode_debug/p3_mounted_pose_coupled_20260928_{3,4,5}/summary.json`, all passed the defined measured free-space scenario. Final status was `HOLDING/GOAL_REACHED`; final TCP errors 10.68/9.98/9.98 µm; measured TCP motion during five seconds of vehicle drive 0.2168/0.2121/0.2151 m; independent Gazebo chassis motion 0.6082/0.6071/0.6071 m. Post-stop displacement 0.00345/0.00202/0.00048 m; final one-second max speed 0.000231/0.000228/0.000230 m/s, below the inherited 0.03 m/s threshold. Zero LCP/crashes, both observers valid, final pose node control state healthy. Result: ACCEPTED C PASS for this defined free-space coupled insertion scenario. It does not clear the historical severe drift, barrier-contact LCP, feedback dropout or unproven jitter budgets; those remain open.

Next: rerun A three times on the current `MoveItIKProvider`/route code because its earlier three-run evidence predates the base-frame and collision-diagnostic edits. Then run final targeted/full CTest and address the analytical benchmark failure without loosening a configured safety limit. Continue vehicle long-hold/reliability and feedback-dropout isolation; D still awaits operator Auto mode and E web login/program identity.

## Iteration P3-A04 / P2-CTest, 2026-09-28

After the mounted base-relative MoveIt and route diagnostics edits, A was rerun three consecutive times with fresh gzserver: `docs/ode_debug/p3_pose_arm_current_20260928_{1,2,3}/summary.json`. All reached joint and pose targets through the position bridge, final `HOLDING/GOAL_REACHED`, independent Gazebo wrist samples 572/566/558 with measured displacement, simulation clock >28 s, observer valid/clean, zero LCP/crash. Result: ACCEPTED A PASS for defined joint+pose free-space scenario on the current built code. The full 68-test FR10 CTest suite then reported 0 failures and one intentional skip; evidence `build_grouped/fr10_pose_controller/Testing/Temporary/LastTest.log`. The previously intermittent analytical benchmark passed this time with measured p99 6.924 µs against its source-defined 85 µs threshold; earlier 88–89 µs failures remain a contention/reliability investigation, not silently waived.

Next: instrument joint-feedback source and callback receive gaps plus handoff drops in `RosControlBackend`, because one mounted run stopped on a 51.753 ms sample while independent observer showed separate callback congestion. Keep 50 ms hardware safety timeout intact. Build, run CTest, then targeted Gazebo fault/timing regression; subsequent source edits will require refreshing A/C three-run evidence before final acceptance.

## Iteration P2-Feedback01, 2026-09-28

Observation: `p3_mounted_collision_pairs_20260928_2` stopped after one 51.753 ms feedback sample; another run's separate Python observer showed a 299 ms receive gap with only 8 ms source gaps, which does not prove the pose node itself was stalled. Hypothesis: distinguish publisher/source gaps, callback scheduling, invalid/replayed samples and buffer handoff before changing the watchdog. Candidate instrumentation added to `RosControlBackend` and `motion_adapter.cpp`: atomic max source/receive gaps, accepted callback count, rejected stamp/sample counts and existing buffer dropped-write count in non-tick status and failure log. No timeout, safety condition, controller command or 125 Hz algorithm changed. Result: PENDING validation; not yet an accepted fix or diagnosis.

Next: build `fr10_pose_controller`, run CTest, then a fresh mounted no-drive or coupled Gazebo run. Record new feedback metrics. If dropout recurs, compare source gap vs receive gap vs buffer drops; keep 50 ms fail-closed behavior and isolate the responsible layer. Because this source change affects A/C runtime, refresh their final three-run evidence after instrumentation is accepted.

## Iteration P2-Feedback02, 2026-09-28

The feedback instrumentation built without errors; full FR10 CTest again had 0 failures of 68 and one intentional skip. Fresh current-build coupled mounted run `docs/ode_debug/p2_feedback_mounted_20260928_1/{summary.json,runtime.log}` reached `HOLDING/GOAL_REACHED`, actual chassis 0.6071 m, simultaneous TCP 0.2156 m, final speed 0.000231 m/s, zero LCP/crash. Node status recorded 4,518 accepted feedback callbacks, max source gap 8.000 ms, max receive gap 10.255 ms, zero buffer dropped writes, zero rejected stamps/samples and max feedback age 10.193 ms. Result: ACCEPTED instrumentation behavior and one coupled runtime; REJECTED any claim that the intermittent 51.753 ms dropout is resolved by a clean run. No watchdog change. This source edit invalidates final A/C three-run repetition sets until refreshed.

Next: audit remaining `motion_adapter.cpp` ownership and select one coherent extraction with deterministic tests; do not change public command or safety behavior. For feedback fault isolation, use the new metrics during repeated startup and controlled callback-load cases. After implementation stabilizes, rerun 3 fresh A and C, verify B graph scope, and run full build/CTest. D still requires SimMachine operator Auto and E requires Web login/program identity.

## Iteration P4-StopReason01, 2026-09-28

Architecture audit found a concrete state-ownership race: the planning worker writes `stop_reason_` directly at seven rejection sites; after a fail-closed `requestStopAll(COMM_FAILURE)` these writes could replace the latched cause. Candidate decision DR-05: keep the current single stop latch and introduce a ROS-free CAS rule for worker-only outcome publication, allowing updates from NONE/GOAL_REACHED/IK_FAILURE but preserving active controlled-stop and safety reasons. `stop_reason_policy.hpp` and `test_stop_reason_policy.cpp` added; only seven worker writes changed to the rule. The control runner remains the sole `MotionExecutor` writer, and no command, safety limit or SDK route changed. Result: PENDING build/test/runtime regression, not yet accepted. Rollback this candidate if fault precedence or 125 Hz timing regresses.

Next: finish the FR10 build and focused stop-reason/realtime tests, then run full CTest and one current-build A/C runtime check. If accepted, final three-run A/C refresh is required again. Continue method/state audit before any planning-service extraction.

## Iteration P4-StopReason02, 2026-09-28

The stop-reason ownership candidate built, focused tests `stop_reason_policy`, `reference_lag_stop` and `realtime_buffer` passed, and full FR10 CTest had 0 failures of 69 with one intentional skip. One fresh current-build A `docs/ode_debug/p4_stop_reason_arm_20260928_1/summary.json` and one C `docs/ode_debug/p4_stop_reason_mounted_20260928_1/summary.json` passed with actual Gazebo motion, goal reach, zero LCP/crash. Matched coupled workload before/after extraction: loop 124.998 Hz in both, zero tick deadline misses and feedback buffer drops; max tick execution 0.503 ms before, 0.219 ms after. This comparison shows no measured timing regression in these two runs, not a proven p99 budget. Result: ACCEPTED DR-05 for preserving latched safety fault causes against late worker rejection; it is a bounded ownership correction, not full adapter decomposition. New source edit again invalidates final A/C three-run repetitions.

Next: add production-plugin B twist scenarios (reverse/lateral/rotation/combined) with independent Gazebo pose, command, wheel/odom and timing evidence; classify directional thresholds with provenance. Continue audit of planning-worker and pose coordinator boundaries. When implementation stops changing, refresh final A/C three-run sets and all build/test gates.

## Iteration P2-BMatrix01, 2026-09-28

First production-swerve case-harness attempt `docs/ode_debug/p4_swerve_forward_20260928_1/{summary.json,runtime.log,manifest.json}` is INVALID: Gazebo controller and independent transport observer ran for 86.8 s simulation with zero LCP/crash, but the Python ROS observer received zero `/clock`, joint, odom or TF samples and therefore never sent a drive command. Root cause in the new harness: child launch env used ROS_DOMAIN_ID=96 while its in-process rclpy observer retained the parent domain. `tools/run_swerve_case.py` now sets the process domain before `rclpy.init`; no production code changed. Result: REJECTED harness readiness implementation, not a vehicle motion failure.

Next: rerun forward under a new label with fresh gzserver and require wheel/odom/clock/controller readiness before command. If valid, run reverse/lateral/rotation/combined cases with independent transport truth and retain each raw result; directional magnitude budgets remain PROVISIONAL.

## Iteration P2-BMatrix02, 2026-09-28

The corrected-domain forward run `docs/ode_debug/p4_swerve_forward_20260928_2/{summary.json,ros_trace.json,vehicle/transport_state.jsonl}` had all observers, one active swerve plugin, actual +0.59873 m x travel, final speed 2.27e-9 m/s, zero LCP/crash. However the raw `/cmd_vel_nav` trace contains 32,784 nonzero messages in five seconds (~6.5 kHz) because the new harness published on every `spin_once` iteration and callbacks made the loop return immediately. Result: INVALID for the planned 20 Hz command-cadence case, although its physics and controller observation remain useful. This is a harness defect, not evidence that the swerve plugin generated those commands. The client publisher now uses monotonic 0.05 s scheduling independently of callback rate; production files unchanged.

Next: rerun forward with fresh gzserver, verify ~100 source-stamped normalized commands in five seconds and no extra nonzero command after stop. Only then proceed to reverse/lateral/rotation/combined diagnostics.

## Iteration P2-BMatrix03, 2026-09-28

The fresh 20 Hz forward rerun `docs/ode_debug/p4_swerve_forward_20260928_3/{summary.json,ros_trace.json,manifest.json}` is diagnostic-valid: 100 nonzero normalized commands over five seconds plus one zero, median wall interval 50.247 ms, no later nonzero command; independent Gazebo dx=+0.59993 m, odom endpoint error ~0.0001 m, final speed 2.20e-9 m/s, zero LCP/crash. The seven-case production-plugin catalogue then ran with a fresh gzserver per case: zero `p4_swerve_zero_20260928_1`, forward `_forward_..._3`, reverse `_reverse_..._1`, lateral `_lateral_..._1`, diagonal `_diagonal_..._1`, rotation `_rotation_..._1`, combined `_combined_..._1`. All seven have `diagnostic_valid=true`, finite wheel/steer state, 101 normalized commands (100 nonzero for moving cases), odom and TF samples, independent Gazebo pose, zero LCP/crash and inherited final-speed/tilt gates. Measured (dx,dy,yaw): zero (~0.00008,0.00014 m,~0); forward (+0.5999,~0 m,~0); reverse (-0.6000,~0 m,~0); lateral (+0.0071,+0.5709 m,+0.0084 rad); diagonal (+0.4284,+0.4158 m,~0); rotation (~0,~0,+0.7430 rad); combined (+0.4236,+0.3531 m,+0.4902 rad). Final odom-vs-Gazebo planar error ranged ~0.0001–0.0060 m; TF translation matched odom at final sample. Result: ACCEPTED as expanded directional/physics diagnostic coverage, not new numerical motion acceptance: minimum displacement/yaw and allowable odom error remain PROVISIONAL. The earlier high-rate forward attempt remains INVALID for the intended cadence.

Next: add expanded URDF and installed plugin hashes/process identity to future run manifests; current results already have source/config/world hashes and raw logs but not all artifact identity fields from P1. Then run focused startup, command-timeout and callback-dropout cases, plus obstacle contact isolation. Preserve prior severe drift and startup crash. Final 3-run A/C still require refresh on the latest stop-reason code; B's earlier three-run forward gate remains scoped to its unchanged vehicle graph.

## Iteration P2-BMatrix04, 2026-09-28

The first artifact-expanded forward run `docs/ode_debug/p4_swerve_manifest_forward_20260928_1/{summary.json,runtime.log,manifest.json}` is INVALID: controller/ROS feedback, 101 normalized commands and expanded URDF were collected, but the independent Gazebo transport observer exited with `pose transport messages=0 target_samples=0`. No ground-truth pose can be inferred from odom or simulator survival. Hypothesis: observer was started before Gazebo master became available. The harness now waits for master TCP readiness before observer startup; the observer flushes its first target pose, and the harness requires that sample before sending any `/cmd_vel`. These changes affect only measurement tools.

Next: run forward with fresh gzserver and verify the new master/first-pose readiness gates plus installed binary/URDF hashes. If valid, use the revised manifest for final B repetitions; retain this no-ground-truth run as INVALID.

## Iteration P2-BMatrix05, 2026-09-28

Observation: manifest-complete fresh production-swerve forward runs `docs/ode_debug/p4_swerve_manifest_forward_20260928_{2,3}/summary.json` were diagnostic-valid. The next unchanged attempt `_4` is INVALID for motion acceptance because gzserver died before controller readiness, independent pose and drive measurement. It is a genuine startup failure: `runtime.log` records gzserver exit -11 after robot spawn, 14 `/clock` samples, no odom/wheels/Gazebo pose, zero LCP. The abnormal exit breaks B's consecutive-run sequence.

Crash evidence: `/var/crash/_usr_bin_gzserver-11.10.2.1000.crash` has Tgid/Pid **47616**, matching `_4/runtime.log` exactly, with `Date: Mon Sep 28 10:23:48 2026`. Its SIGSEGV stack is `libgazebo_ode.so` → `dSpaceCollide2` → `gazebo::physics::ODEMultiRayShape::UpdateRays` → `gazebo::sensors::RaySensor::UpdateImpl` on the sensor thread. The vehicle model contains one `<sensor type="ray" name="laser">` in `src/vehicle_description/vehicle_description/urdf/body_sensors.xacro`; camera log lines immediately before failure are temporal context, not proof the camera caused it. Result: ACCEPTED identification of the crashing call path; the underlying lifetime/concurrency cause remains unproven. The failed startup is not upgraded to PASS. No production code or physics parameter was changed.

Rejected hypotheses and reasons: no evidence supports treating a successful rerun or zero LCP as proof of startup stability; the stack points to the ODE multiray sensor path, not controller commands. Files modified this iteration: `PROGRESS_LEDGER.md`. Files not fully regression-validated: existing pre-ledger tree plus P1/P3/P4 changes and the B harness; A/C latest-code three-run refresh remains. Build/test status unchanged since P4-StopReason02 (FR10 69 CTests, zero unexpected failures, one intentional skip).

Next: in a generated diagnostic-only vehicle model, omit only the Gazebo `<sensor type="ray" name="laser">` while keeping LiDAR link/collision/inertial, cameras, controllers, world and spawn unchanged. Run matched fresh startup trials with original and omitted-ray model using `tools/run_swerve_case.py` and save manifests, raw logs, independent pose and crash dumps. Set the reliability repetition count as PROVISIONAL until supported by a requirement/rationale; do not infer a production fix from one clean trial. If the crash recurs without the ray, reject the ray-specific hypothesis. If it only occurs with ray, investigate sensor/ODE lifecycle and select a reversible production correction after compatibility audit. Then run A/C three current-build fresh repetitions and final build/tests. D requires operator Auto; E requires authenticated program identity.

## Iteration P2-BMatrix06, 2026-09-28

Controlled ray-path isolation used `tools/run_swerve_case.py` with an automatically generated launch, preserving the source launch except one injected DOM change. No production model/controller/world was edited. The no-ray run `docs/ode_debug/p2_ray_isolation_noray_20260928_1/summary.json` passed measured forward/settling with 479 independent Gazebo poses, 0 LCP/crash; the generated expanded URDF still contains the physical LiDAR link and camera but no Gazebo ray sensor. The matched production-ray control `_control_..._1` crashed after spawn with gzserver exit -11 and no pose/drive measurement. Ten older diagnostic no-ray startup runs recorded 0 crashes; an older with-ray no-state-plugin set had one crash in ten. This supports the CPU ray path as a startup trigger, not a proven fix or exact race mechanism.

A generated GPU-ray candidate `p2_ray_isolation_gpu_20260928_1` ran without crash and published 248 `/scan` messages, but **all 500 ranges were nonfinite in all 248 messages** despite nearby world obstacles. Its initial `diagnostic_valid=true` summary predates the added finite-return gate and is not a valid LiDAR acceptance result. Result: REJECTED GPU ray candidate for this headless setup; no production switch.

Generated CPU ray `always_on=false` run `p2_ray_isolation_defer_20260928_1` published 254 scans with 181–185 finite rays, moved ~0.60 m and had zero LCP/crash. The next identical run `_2` is INVALID for motion and reveals a separate **gzserver abort -6** during controller startup: `rclcpp::exceptions::RCLError: failed to send response: cannot publish data`. It had valid finite scans but no odom/drive. The old harness `gzserver_crash` regex caught only -11, so `_2/summary.json` misleadingly says `gzserver_crash=false`; raw log and `launch_alive_at_measurement=false` prove abort. The harness now detects any logged abnormal gzserver exit and limits `ListControllers` to one in-flight request every ≥0.5 s. Fresh deferred `_3` and with-ray control `_control_..._2` both passed after this harness change, with ~251 valid 500-ray scans and 3 controller queries; these successes do not erase the earlier failures.

A further generated deferred-ray test `_late_20260928_1` created the `/scan` subscriber only after controller readiness. It passed: 239 scans with finite returns, first scan source time 186.453 s after first observed clock 182.703 s, actual ~0.60 m motion, zero LCP/crash. This supports that scan output can activate after readiness without loss of frame/range contract. Repeats `_late_..._{2,3}` are running and are not yet classified. All new LiDAR scan-validity checks (topic sample continuity, 500 ranges from Xacro, correct frame and at least one finite return in obstacle-containing world) are measurement validity rules, not scan-performance budgets.

Result: ACCEPTED ray-path trigger evidence and improved crash classification; REJECTED GPU-ray substitution; deferred CPU ray remains a candidate pending repeated startup and independent service-abort isolation. Files modified: `tools/run_swerve_case.py`, `PROGRESS_LEDGER.md`. Files not fully regression-validated: this harness and all prior working-tree edits. No new production build required yet; source model unchanged.

Next: classify `_late_..._{2,3}` from raw summaries/logs. If valid, compare more matched startup attempts using bounded service polling, then decide whether to add a delayed-activation production launch path. Do not silently disable `/scan` or claim startup reliability from 3 runs when prior with-ray crashes and separate -6 abort remain. Investigate the -6 service response separately, then run 3 current-build A/B/C fresh scenarios with real scan evidence; D operator Auto and E login/program remain external gates.

## Iteration P2-BMatrix07 / P3-Model01, 2026-09-28

The deferred CPU-ray diagnostic with subscriber only after controller readiness passed three consecutive fresh runs `docs/ode_debug/p2_ray_isolation_late_20260928_{1,2,3}/summary.json`, each with ~239–240 real 500-ray scans and finite returns, independent ~0.60 m chassis motion, zero LCP/crash. Decision DR-06 (simulation LiDAR): apply only `always_on=false` to the existing CPU ray in `src/vehicle_description/vehicle_description/urdf/body_sensors.xacro`; preserve link/collision/inertial, scan geometry, 10 Hz, `laser_frame`, `/scan`, camera, world and controller. The change is reversible if scan or startup/coupled regression fails. `vehicle_description` and `vehicle_bringup` built; vehicle_bringup CTest 3/3. Three consecutive fresh production B forward runs `docs/ode_debug/p2_prod_defer_forward_20260928_{1,2,3}/summary.json` passed controller, wheel/odom, independent Gazebo pose and settling with 0 LCP/crash. Recomputed scan and model evidence in `docs/ode_debug/p2_prod_defer_forward_20260928_assessment.json`: 251/256/266 scans, finite returns in all; normalized URDF hash identical across runs after replacing only the generated temporary controller-YAML path. This is B free-space evidence; it does not resolve the prior RaySensor SIGSEGV mechanism or separate rclcpp service abort -6. The harness now requires real finite `/scan` for non-isolation runs and records canonical URDF and Xacro source hashes going forward.

**Explicit user correction, highest priority:** A standalone and C mounted must use one FR10 arm model: `vehicle_description/urdf/fr10_arm.urdf` plus `fr10_wrist.xacro` and the same `charging_tcp`; standalone may omit vehicle links and add only the fixed world base. The previous A harness launched `fr10_pose_controller` vendor Gazebo model with `base_link`/`tool0`, while C used `fr10_base_link`/`charging_tcp`. Even the three freshly completed `p2_final_arm_20260928_{1,2,3}` are therefore historical **wrong-model A** evidence and cannot count toward A acceptance, despite their old summary `pass=true`. No further vendor-model Gazebo run is authorized by the current user instruction. This does not delete vendor files needed by other workflows.

Decision record DR-07: select the existing shared arm Xacro composition via `vehicle_bringup/simulation.launch.py` with `enable_vehicle:=false, enable_arm:=true, arm_controller_profile:=pose` for A, and the same launch with both enabled for C. Audit confirms `arm_only.xacro` and `complete_visual.xacro` include the same `fr10_arm.urdf` and `fr10_wrist.xacro`; A/C should share base frame `fr10_base_link`, TCP `charging_tcp`, pose-node parameters and collision exclusions where valid. Before acceptance, compare expanded arm link/joint/tool signature A vs C and observe the actual position controller in both. Revert integration if semantic robot/frame or pose control fails; do not revert to vendor Gazebo arm.

Result: ACCEPTED B scoped free-space candidate; P3 model unification PENDING implementation. Files modified: `src/vehicle_description/vehicle_description/urdf/body_sensors.xacro`, `tools/run_swerve_case.py`, A/C harnesses for abnormal Gazebo exit and C scan capture, this ledger. Files not fully regression-validated: all model/harness edits until shared-model A/C and broader physics/reliability run. Build/test: two model/bringup packages passed, vehicle_bringup CTest 3/3; prior FR10 69 CTests unchanged. A=NOT RUN on required model, B=scoped PASS, C=requires rerun on latest model, D=FAIL pending operator Auto, E=NOT RUN/login gated.

Next: ensure no old A process remains; change `tools/run_fr10_pose_gazebo_smoke.py` to launch the shared arm-only composition through `vehicle_bringup`, use `fr10_base_link`/`charging_tcp` and Gazebo entity/world from that launch. Add a deterministic expanded-model identity check for arm geometry/joints/TCP in A vs C. Run one A smoke and one C smoke, then three fresh A/C repetitions if valid. Do not run the vendor-model Gazebo profile again. After model consolidation, continue startup -6/obstacle/dropout/timing investigation and D/E external gates.

## Iteration P3-Model02, 2026-09-28 — in progress

Observation: A/C Gazebo expand `vehicle_description/urdf/complete_visual.xacro`, but its `fr10_arm.urdf` is only an include of `fairino_description/urdf/fairino10_v6.urdf`. FakeSystem/physical/MoveIt still expand `fairino10_v6_moveit2_config/config/fairino10_v6_robot.urdf.xacro`, whose D455/placeholder wrist and SRDF use `base_link`/`tool0`. The active physical/pose default config also retains these frames. This violates the user's one-source requirement.

Decision record DR-08 (candidate, now selected from dependency audit): own the FR10 core URDF, seven meshes, D435/CCS2 wrist and SRDF in the `vehicle_description` package. Retain a single robot geometry entry point, `complete_visual.xacro`, parameterized for vehicle presence; add an optional ros2_control declaration for FakeSystem/FAIRINO without duplicating geometry. Keep Gazebo's existing control composition and FAIRINO's sole hardware plugin and safety gates. MoveIt configuration remains a configuration package and points at these description-owned files. Remove the old D455/placeholder/tool0 model and obsolete description package only after all consumers have migrated and expansions/builds pass. Roll back this iteration if the six arm joint/inertia/limit values change, A/C model identity breaks, multiple j1–j6 command owners appear, or ROS/MoveIt/hardware plugin selection changes unexpectedly.

Provenance: this model convention and deletion are explicitly requested by the user. No numerical safety/acceptance threshold is changed. Tests planned before final verdict: source contract RED/GREEN; pre/post canonical arm mechanical signature; Xacro expansion for arm-only/mounted/mock/FAIRINO; MoveIt semantic group and link validation; package build/CTest; fresh measured A/C Gazebo runs. D is operator Auto gated and E login/program gated; no physical command is authorized.

Current status: PENDING implementation. A and C historical evidence is invalidated for the final source-model revision. B scoped free-space evidence remains valid if its vehicle geometry/control source is unchanged. Next: capture pre-edit arm mechanical signature, run the source contract RED, move FR10 source and meshes into vehicle_description, migrate active launch/config consumers, remove old files only after all references are gone, then build/test and execute current-model A/C regression.

### P3-Model02 experiment M02a — single description source and first runtime, 2026-09-28

The FR10 seven rigid-core links, six moving joints, seven STL meshes, D435/CCS2 wrist and `fr10.srdf` now live under `src/vehicle_description/vehicle_description`; old `fairino_description` source/build/install package and MoveIt vendor URDF/SRDF/ros2_control Xacros were removed. `complete_visual.xacro` is the sole geometry entry point for A/C/GenericSystem/FAIRINO/direct Gazebo. The Gazebo base collision cylinder is now an explicit simulation parameter in that same source rather than an untracked launch DOM mutation; physical/fake retains the original mesh. The pre-edit 13-element mechanical hashes and seven mesh SHA256s are in `docs/model_migration/before.json`. `docs/model_migration/{A,C}_gazebo.urdf` pass `tools/check_fr10_model.py --compare`. The new source contract ran RED (old copies/dependencies), then GREEN 6/6; description CTest 1/1, bringup CTest 4/4, FR10 CTest 70/70 with one intentional skip. Five affected packages built. These are static/build results, not runtime acceptance.

Fresh A `docs/ode_debug/p3_unified_model_A_20260928_1/summary.json` PASS for a joint+pose smoke: canonical live model signature `6273ab89...`, 690 independent Gazebo wrist samples, 35.8 s simulation time, 0 LCP/crash, active position controller. Fresh C `docs/ode_debug/p3_unified_model_C_20260928_1/summary.json` FAIL: vehicle moved 0.6267 m and 1,473 independent vehicle poses with 0 LCP/crash, but pose goal stopped with `REFERENCE_LAG` and no arm position controller active during command. Raw `runtime.log` shows `fairino10_controller` spawner's `ListControllers` timeout; controller activated at 1790570530.604, after pose node started and after stop at 1790570527.134. Measured initial j3 had drifted to 1.60947 rad from home 1.5708. This is a startup ownership race, not a changed safety limit or successful C run. A transient joint-state receive gap reached 59.3 ms while source gap remained 8 ms; the separate dropout issue remains open.

Hypothesis M02b: starting `fr10_pose_controller_node` only after arm controller spawner exits successfully prevents commands before j1–j6 are claimed. RED test `test_pose_node_starts_only_after_arm_controller_spawner_succeeds` failed before implementation, then passed after event sequencing; bringup build and CTest 4/4 passed. Result: candidate PENDING measured C rerun. Revert sequencing if controller startup deadlocks or the measured A/C command path regresses. No safety limit, mechanical transform or SDK route was changed.

Next: run a fresh mounted C on the new launch and inspect active controllers *before* pose publication, initial and settled j1–j6, action outcome, independent Gazebo vehicle/wrist pose, ODE and timing. If C passes, repeat A/C on fresh servers for required consecutive gate; if C still fails, isolate arm hold from vehicle drive while preserving the REFERENCE_LAG guard. D still requires operator Auto; E requires authenticated Web program identity.

### P3-Model02 experiment M02b — startup race resolved for one measured C, 2026-09-28

Fresh C `docs/ode_debug/p3_unified_model_C_20260928_2/summary.json` on the revised launch passed a coupled pose+swerve smoke: arm position controller activation at 1790570877.570 preceded pose node process start, then both arm goals reached; 756 independent vehicle/wrist Gazebo poses, 0.60658 m chassis displacement, 0.20969 m TCP motion during drive, valid finite `/scan`, 40.8 s simulation time, 0 LCP/crash. Arm feedback max source gap 8.0 ms, max receive gap 10.55 ms, 0 buffer drops/deadline misses, measured loop 124.997 Hz. Its live FR10 arm signature `6273ab89...` matches A `p3_unified_model_A_20260928_1`. Result: ACCEPTED startup ordering fix for this one C runtime; the failed C attempt remains in the ledger. A run 1 preceded this launch change and cannot count toward final current-launch three-run sequence. C run 2 is candidate repetition 1/3, subject to no further source changes.

Next: run two more fresh current-launch C coupled repetitions and three fresh current-launch A arm-only repetitions, each with new domain/port and complete independent pose/controller/clock/log evidence. A FAIL/INVALID or C FAIL/INVALID resets only that configuration's consecutive sequence. Compare realtime metrics without inventing p99 budget. Then decide whether B needs refresh from model hash comparison, run final CTest, and retain D/E blockers.

### P3-Model02 C three-run gate — 2026-09-28

Fresh mounted runs `docs/ode_debug/p3_unified_model_C_20260928_{2,3,4}/summary.json` passed consecutively on the same canonical source and launch after the arm-spawner sequencing fix. Every run had a new gzserver, live `complete_visual`/FR10 signature prefix `6273ab896f3f`, active swerve+position controllers, arm pose goal reached, 0 LCP/crash and independent vehicle/wrist Gazebo samples. Measured chassis displacement was 0.60658/0.60633/0.60754 m; TCP during vehicle motion 0.20969/0.20907/0.21619 m. Arm feedback max receive gaps were 33.79/35.17/38.99 ms, with 8 ms source gaps and no final controlled stop. Result: ACCEPTED for C's three-run coupled free-space gate, scoped to this source revision. Obstacle contact, startup reliability and a validated realtime p99/deadline budget remain separate open gates.

Next: run A arm-only three fresh current-launch repetitions, compare live FR10 signature to C and inspect active controller, independent wrist truth, goal status, 0 LCP/crash and clock/timing. Then assess B source-hash validity and run full current-code build/CTest.

### P3-Model02 A three-run gate and observer INVALID — 2026-09-28

A run `docs/ode_debug/p3_unified_model_A_20260928_2/summary.json` reached joint/pose goals with 0 LCP/crash but received zero independent Gazebo transport samples; it is **INVALID**, not PASS. `transport.log` records `pose transport messages=0 target_samples=0`. The A harness was corrected to wait for Gazebo master TCP and first target pose before publishing; the observer process already flushes its first sample. This is measurement-only sequencing, no model/controller change. Fresh A runs `docs/ode_debug/p3_unified_model_A_20260928_{3,4,5}/summary.json` then passed consecutively: 692/700/691 independent wrist poses, each about 36 s simulation time, joint+pose goals reached, active position controller, zero LCP/crash. Their shared arm signature `6273ab896f3f...` equals all three current C runs. Measured max feedback receive gaps in the A final statuses were 8.567/8.570/8.612 ms. Result: ACCEPTED A's three-run free-space gate for this source revision. The prior observer-invalid run remains retained.

Read-only D/E preflight confirmed current SimMachine container identity `fairino_simmachine:v3.9.8` at 192.168.58.2, ports 80/8080/20005 reachable, no workspace mount; Web `/index.html?...#/process` redirects to `/login.html`. Historical mode=1 Manual gate was measured before this source migration, and prior D runs used `tool0`; no current-model D acceptance is possible without operator Auto and Web identity. No physical motion was attempted.

Next: audit/fix identified P2 swerve command timestamp/frame/replay/clock-reset issue and stale odom timestamp with pure tests, then repeat affected B/C regressions on that candidate. Separately resolve FR10 diagnostics stop-reason ownership and rerun affected A/C. Keep safety limits unchanged; measured p99/deadline budgets are still provisional.

### Iteration P4-StopReason03 — control-owner goal outcome, 2026-09-28

Observation: `diagnosticsTick()` published a cached `GOAL_REACHED` snapshot and unconditionally stored that outcome in `stop_reason_` from the ROS diagnostics callback. A fault or controlled stop latched after the snapshot could therefore be overwritten. Hypothesis: assign goal-outcome writes to the control tick and allow the transition only from `NONE` while no stop is requested or pending.

Experiment: extend the ROS-free `test_stop_reason_policy.cpp` with healthy-goal, latched `COMM_FAILURE`, pending `PATH_DEVIATION`, shutdown and non-goal cases. The test first failed compilation because `recordGoalReachedStopReason` did not exist. Add the CAS policy in `stop_reason_policy.hpp`, invoke it after command-write success in `controlTick()`, and remove all three goal-reason stores from diagnostics. Diagnostics logs a goal only when the reason read is `GOAL_REACHED`. No safety limits, model, public topic or SDK path changed.

Evidence: direct RED compiler diagnostic and GREEN executable result; `colcon build --packages-select fr10_pose_controller` finished; `ctest --test-dir build_grouped/fr10_pose_controller --output-on-failure -j2` passed 70/70 with one intentional skip. Result: ACCEPTED as a bounded ownership fix; runtime A/C regression is still required because this changes status behavior. A and C previous three-run outcomes remain historical for the prior source revision and do not count as final current-code repetitions. Files modified: `src/fr10_arm_control/fr10_pose_controller/include/fr10_pose_controller/stop_reason_policy.hpp`, `src/fr10_arm_control/fr10_pose_controller/src/motion_adapter.cpp`, `src/fr10_arm_control/fr10_pose_controller/test/core/test_stop_reason_policy.cpp`, and this ledger. Files modified but not yet fully regression-validated: these three FR10 files plus the pre-existing dirty tree.

A: NOT RUN on this source revision. B: prior scoped PASS, pending swerve changes. C: NOT RUN on this source revision. D: INVALID/current-model run pending operator Auto; E: NOT RUN/login and program identity pending. Web and physical hardware were not touched.

Next: review the P2 swerve command freshness/odom timestamp patch and its scoped tests. When both FR10 and swerve candidates are frozen, rebuild affected packages and run one fresh A, B and C gate to catch integration regressions. If valid, complete three consecutive fresh A/B/C repetitions with source hashes and independent Gazebo pose; any FAIL/INVALID resets that configuration's sequence. Preserve fault/observer failures. D requires an operator to select Auto in the verified SimMachine; E requires authenticated FR10 program identity.

### Iteration P3-SceneState01 / P1-Geometry01 — 2026-09-28

Observation: `MoveItIKProvider::solve/validate()` initialized a fresh RobotState to defaults, then set only the six arm joints. On mounted C, collision validation therefore ignored the PlanningScene's measured steering/wheel state. Separately, simulation loaded `vehicle_geometry.yaml` but did not compare it against the independently configured swerve geometry and expanded URDF.

Experiment P3-SceneState01: extend the existing collision fixture with a non-arm prismatic vehicle link that moves into the tool collision volume when its scene variable changes. RED: `moveit_collision_provider` failed with `moving_vehicle_collision_rejected=0` while earlier collision tests passed. Change `MoveItIKProvider` to copy the pinned/live scene current state before replacing only j1–j6; use the same pinned state for route-contact diagnostics. GREEN: scoped test passed; FR10 build and full CTest 70/70 passed, one intentional skip. Result: ACCEPTED for collision queries at planning-snapshot time. The fixture proves a non-arm joint can affect collision; it does not yet prove freshness of every mounted joint or continuous replanning while the vehicle moves. Roll back if A/C runtime or timing regresses. Files: `moveit_ik_provider.{hpp,cpp}`, `motion_adapter.cpp`, `test_moveit_collision_provider.cpp`, ledger. A/C previous source-revision repetitions are invalidated.

Experiment P1-Geometry01: prelaunch `validate_vehicle_geometry()` now compares effective swerve controller YAML against `vehicle_geometry.yaml` and the expanded wheel steering/axle origins, axes and collision radii. Mismatch raises before spawn; the numeric `1e-12` comparison tolerance is solely XML decimal serialization tolerance, not a motion threshold. The controller YAML remains for standalone consumers. Agent ran RED mutation test, GREEN six cases, `vehicle_bringup` build and CTest 5/5. Result: ACCEPTED as a launch/config consistency guard, pending B/C runtime. Files: `src/vehicle_bringup/launch/simulation.launch.py`, `CMakeLists.txt`, `test/test_vehicle_geometry_contract.py`, ledger. Roll back if a semantically equivalent valid geometry expansion is wrongly rejected or B/C startup regresses.

A/B/C: NOT RUN on the combined current candidate. D: INVALID/current-model motion pending operator Auto. E: NOT RUN/login and program identity pending. Files still awaiting full runtime validation include the swerve freshness patch, MoveIt scene-state fix, geometry guard and TCP source-stamp change. Next: build FR10 after TCP source-stamp change, run one current A smoke and compare TCP header source stamps against joint-state stamps and independent Gazebo wrist truth; then freeze swerve freshness patch and run one B/C smoke before three consecutive fresh A/B/C repetitions. Record realtime distribution without using unproven p99 budgets.

### Force-position implementation / MuJoCo + charging-port continuation — 2026-10-01

Artifact root: `test_results/fr10_force_control_20261001T035857Z/`.
Status: IN PROGRESS; not a deployment acceptance. Force remains disabled in
`controller.yaml`. No physical robot contact, Auto switching, enabling or alarm
reset has been performed. Dirty worktree preserved; no `.orig` restoration.

- Pure shared force library, insertion session/FSM/action and online joint OTG
  path are implemented. New measured geometry monitor checks TCP depth, 2 mm
  lateral bound and configured orientation bound; wrench rotates from moving
  TCP into fixed insertion axes. Raw and conditioned torque limits are both
  checked. Gravity compensation now requires explicit calibration and rotates
  gravity into sensor axes. Tare mutation is queued to the control owner.
- Split node wiring, motion runtime, planning worker, command router,
  diagnostics and force runtime into separate translation units. Initial
  extraction build failures were corrected; no claim of acceptance was made
  from those failed builds. Source-stamp regression detected a startup TCP
  sample with no valid feedback and a receipt-time stamp; source-stamp guard
  fixed this. `gazebo/legacy_final_01/summary.json` PASS: JOINT, POSE, HOLD,
  sole JointGroupPositionController, source stamp match, no ODE LCP/crash,
  125.010 Hz, max tick execution 0.088 ms, zero tick deadline misses.
- MuJoCo Python 3.14.0 is installed. `mujoco/normal_06` and
  `mujoco/normal_post_refactor` PASS: independent depth 4.88449 mm for a 5 mm
  target, final axial raw load 9.733 N for 10 N, completion then HOLD.
  Seven fault runs `fault_{dropout,duplicate,nan,overload,torque,lost_contact,cancel}_01`
  PASS with completed controlled stop and no false success. These are proxy
  spring contact/wiring tests, not CCS2 pin mating or ros2_control plugin tests.
- `mujoco/corridor_validated_01` PASS: worker log shows 52 axial IK/collision
  samples and 550 sampled Ruckig braking states. Scoped scene fixture/pair
  validation was added afterward; fresh MuJoCo/Gazebo runs are ongoing.
- Gazebo spring-contact fixture uses real gravity, the native FT joint plugin
  and production GazeboSystem/JointGroupPositionController. `gazebo/contact_01`
  FAIL (planning result handoff on timing-stop replan); retained, fixed and
  rerun as `contact_02`. No full-contact acceptance yet.
- CTest after safety build: motion core 2/2, force 2/2, insertion 3/3, pose
  70 entries with one intentional numerical-IK skip, no failures. Model,
  MuJoCo and source-stamp Python tests 13/13. Final source freeze/retest pending.
- Charging port STL unchanged, SHA256
  `6f5bd5f58cf5583c3616ef552eec23280913977fb5555a5c7a9196aee039e7aa`.
  `charging_port_tcp` added to model SDF; simulator visuals use the supplied
  mesh with scale 0.001 and original +5 mm mesh offset. Frame alignment is
  established, but JLR connector/CAD port pin pitches differ; no physical
  mating PASS is claimed. Viewer replays recorded MuJoCo data only, emits no
  ROS or hardware commands. Space pauses; C toggles full-arm/close-up camera.

Reproduce:

```bash
source /opt/ros/humble/setup.bash
source ros2_current/install/setup.bash
python3 tools/run_fr10_force_mujoco.py --backend mujoco --scenario normal \
  --domain 195 --output test_results/force_mujoco_fresh
python3 tools/run_fr10_force_mujoco.py --backend gazebo --scenario normal \
  --domain 197 --port 11997 --output test_results/force_gazebo_fresh
python3 tools/view_fr10_force_mujoco.py --close-up \
  --trace test_results/fr10_force_control_20261001T035857Z/mujoco/normal_06/trace.jsonl
```

Next: full contact completion/retract/fault evidence in Gazebo, fresh fault and
scene-obstruction matrix, mounted legacy regression, SimMachine readiness and
version checks, source/binary/config manifest, final report and real sensor
commissioning checklist. Keep force disabled until all required gates pass.

### Force-position handoff — final current-source gates, 2026-10-01

Artifact: `test_results/fr10_force_control_20261001T035857Z/REPORT.md`.
Disposition: simulation software/proxy handoff complete; full deployment plan
NOT accepted. Default force OFF; additional FAIRINO force lock retained. No real
contact, automatic Auto/enable/reset or image replacement occurred.

- Phase 0/1: continuation baseline and dirty-tree provenance in `BASELINE.md`;
  no fabricated clean pre-change snapshot. Node wiring/runtime/planning/router/
  diagnostics/force ownership extracted; collision provider moved to planning
  with compatibility header. No `.orig` recovery. Source-stamp/startup feedback
  regression fixed and measured again.
- Phase 2/3: pure six-DOF admittance, wrench filter/transform/compensation,
  contact/safety/FSM, Insert.action and online seeded IK/Ruckig implemented.
  SO(3) reflection/duplicate-row RED test found insufficient transform validation;
  orthogonality + determinant checks fixed it. New variable-dt/limits/filter unit
  tests passed. Last force HOLD now waits for acknowledged controlled deceleration
  while safety stays active, freezes integrator and only then returns SUCCESS.
  Normal tests also verify 1 s unchanged commands and in-tolerance measured depth.
- Build: four affected packages PASS (`unit/hold_ack_build.log`). CTest motion
  core 2/2, force 2/2, insertion 4/4, pose 70 entries with 1 intentional numerical
  IK SKIP, no failures. Planning package registers no separate CTest; its collision
  wrapper is covered in pose tests. Python model/bridge/feedback tests 21 PASS.
  Actual force+FR10 analytical IK/FK 50,000-cycle benchmark: mean 14.9344 µs,
  p99 19.367 µs, max 51.268 µs (`unit/handoff_actual_ik_benchmark.log`), not SDK soak.
- Hardware CTest is NOT all green: cppcheck/xmllint PASS, lint_cmake/uncrustify
  FAIL. Eight unchanged CMake style errors and 16 files with formatting divergence
  remain recorded; no vendor-wide reformat or disabled lint. Health schema-v2
  packet capacity now matches its 20 published fields; native ServoJ route retained.
- Phase 4/5: final selected matrix 50/50 PASS: Fake 16, Gazebo 17, MuJoCo 17.
  Includes normal/HOLD, retract, dropout/duplicate/NaN/raw overload/torque/lost
  contact, cancel, scene/base motion, forbidden obstruction/missing fixture,
  no-contact signal/timeout, backward clock; Gazebo/MuJoCo pause/resume included.
  Artifacts: each backend `hold_ack_normal_01` + `handoff_*`. Native Gazebo FT,
  real gravity and scoped spring-proxy contact exercised. Global sim clock jump
  reports abort with stop UNKNOWN, never a false controlled-stop confirmation.
- MuJoCo dependency block is superseded: Python 3.14.0 installed, canonical plant
  runs. Topic-based joint position plant only, NOT a MuJoCo ros2_control plugin.
  CAD is visual-only and full arm physical collisions are not modeled in this
  proxy; MoveIt worker collision checks remain. Full material/misalignment/friction
  campaign and matched CCS2 pin mating remain pending, not counted as PASS.
- Current legacy regression: Gazebo standalone `legacy_handoff_01` PASS: JOINT,
  multi-joint, POSE/HOLD, sole JGP owner, source stamps match, zero LCP/crash,
  124.993 Hz, max tick 0.065 ms, zero execution deadline misses. Mounted
  `mounted_handoff_01` PASS: pose ~10 µm error, chassis 0.6041 m, zero LCP/crash,
  124.994 Hz, max tick 0.147 ms. GenericSystem `fake/legacy_handoff_01` PASS for
  JOINT, one-shot POSE and 10 Hz retarget stream; no thresholds widened.
  Failed `mounted_release_01` (54.544 ms feedback watchdog stop) is retained.
  Later PASS does not prove host scheduling/dropout reliability resolved.
- Phase 6: FR5 substitution explicitly authorized by user. Docker-scoped fresh
  `simmachine/fr5_preflight.json`: FR5 model, web v3.7.8/controller V3.7.68,
  digest d5a3dedd…, ports 8080/20003/20004 open, 20005 refused, native RPC -2.
  No valid state/Auto/enable assertion. Static SDK client ABI 8/8 PASS, but no
  native ServoJ rate/gap, compatibility acceptance or cabinet 50,000-cycle soak.
  FR5 is allowed for joint transport; FR10 Cartesian/contact equivalence is not.
- Phase 7: `docs/fr10_force_sensor_handoff.md` records sensor/driver/rate, sign/
  frame/moment arm, tare, calibrated mass/CoG, noise/temperature/freshness/overload,
  known loads, native timing, contact/stopping envelope and operator sequence.
  Real gains/limits not inferred. Physical robot contact NOT RUN.
- Viewer switched to `mujoco/hold_ack_normal_01/trace.jsonl`; replay only, no ROS
  publisher, no robot command. Supplied port STL unchanged, charging_port_tcp
  aligned; different connector/port pin pitches cannot be fixed by rigid alignment.
- `manifest.json` / `effective_config.yaml` generated from source/config/binaries
  and summaries. Selected simulation gate explicitly requires all 50 results;
  earlier failed/invalid runs remain in the full summary inventory. Some Gazebo
  evidence collectors still show SIGINT-time pending-message Python errors;
  no universal clean-shutdown claim is made.

Reproduce (fresh output directories, sequential physics runs):

```bash
source /opt/ros/humble/setup.bash
source ros2_current/install/setup.bash
python3 tools/run_fr10_force_mujoco.py --backend gazebo --scenario normal \
  --domain 194 --port 11994 --output test_results/force_gazebo_new
python3 tools/run_fr10_force_mujoco.py --backend mujoco --scenario pause \
  --domain 194 --output test_results/force_mujoco_pause_new
python3 tools/run_fr10_force_mujoco.py --backend fake --scenario duplicate \
  --domain 194 --output test_results/force_fake_duplicate_new
python3 tools/fr10_simmachine_preflight.py --probe /tmp/fr10_simmachine_readonly_probe \
  --output test_results/force_simmachine_readonly_new.json
python3 tools/fr10_force_artifact_manifest.py \
  test_results/fr10_force_control_20261001T035857Z
```

Next external gate: provide native-SDK-compatible SimMachine (FR5 acceptable)
with realtime state, followed by operator-safe Auto/enable; only then design an
explicit verified-simulator gate without unlocking physical force. Full contact
acceptance additionally needs a matched socket/connector model and commissioned
real sensor/profile. Do not bypass ports/ABI/feedback/geometry barriers to fake a
PASS. Core/proxy implementation is handed off with those limits explicit.

## 2026-10-01 — Grounded charging-port fixture

- User requested the port below the startup connector, with its rear face
  touching the floor. Measured the supplied
  `vehicle_description/meshes/cong_sac/meshes/cong_sac.stl` triangles after
  0.001 scaling and the existing visual/model rotations: port thickness is
  0.0938 m. No mesh or horizontal placement/orientation was changed.
- `fr10_force_home_test.world`: lowered visual and spring fixture origins from
  z=0.616499085374 to z=0.093799987; added the ground plane at z=0. The
  small home roll offset is included in the measured grounding height.
  Spring plate center and 5 mm final insertion goal are now z=0.088799987.
- `fr10_force_test.world`: socket mating-frame z=0.0938 instead of 0.65.
  Its rear collision plate remains 55 mm down local +Z; planning proxy center
  is corrected to world z=0.0388, matching the actual SDF collision.
- Updated the launch-owned planning publisher and user command documentation.
  `force_fixture_profile:=socket` now selects its matching world automatically
  when `force_world` is omitted; an explicit world override remains supported.
  Arm startup joints and all force/controller gains/limits remain unchanged.
- Validation: 36 pytest cases PASS (sole model, backend selection, controller
  ownership); actual transformed CAD minimum z is zero within 0.1 micrometre
  for both profiles and maximum z is 93.8 mm within 1 micrometre.
  Both `gz sdf -k` checks PASS. Launch setup for both profiles produced the
  matching runtime world and valid absolute CAD URI. Python syntax checks PASS.
  The existing symlink-install resolves these changed world/script/launch files
  directly from source, so a launch restart picks up the change.
- No physical contact run was performed for the new near-floor geometry;
  prior elevated-fixture PASS artifacts remain historical and do not certify
  contact at the new position. No robot commands were sent.

Reproduce geometry/config verification:

```bash
source tools/source_simulation.sh
ROS_LOG_DIR=/tmp/vinuni_grounded_fixture_logs python3 -m pytest -q \
  src/vehicle_description/vehicle_description/test/test_single_fr10_model_contract.py \
  src/vehicle_bringup/test/test_backend_selection.py \
  src/vehicle_bringup/test/test_simulation_controller_contract.py
gz sdf -k src/fr10_arm_control/fairino10_v6_moveit2_config/worlds/fr10_force_home_test.world
gz sdf -k src/fr10_arm_control/fairino10_v6_moveit2_config/worlds/fr10_force_test.world
```

## 2026-10-01 — Fix unified force launch clock and verify grounded insertion

- Investigated the user's log: valid 0.540 m / 7.464 s preinsert plan activated,
  but only a few millimetres of motion before action code 19 (CANCELED).
  The public unified Gazebo include omitted the physics-clock parameter file
  already used by force regression launches. Active force sessions pause on
  unchanged sim time; default 10 Hz /clock throttled nominal 125 Hz playback.
- `vehicle_bringup/launch/simulation.launch.py` now passes the existing
  `gazebo_force_clock.yaml` (publish_rate 1000 Hz) automatically for force only.
  Daily launch flags unchanged. Controller math, 8 ms samples, pause semantics,
  50 ms feedback timeout, 1 degree reference lead and force/collision limits
  unchanged. Added actual-launch contract tests for force and legacy branches.
- Build `vehicle_bringup`: PASS. Direct pytest: 38 PASS. CTest: all 7 registered
  vehicle_bringup suites PASS with writable ROS_LOG_DIR. Initial sandbox log-path
  failures were environmental and resolved without changing code.
- Public unified bringup tested in two isolated headless Gazebo runs: both
  complete PREINSERT/TARE/APPROACH/CONTACT/FORCE/VERIFY/SUCCESS/HOLD with code 0.
  TCP moves 0.53294 m down from home; final depth 5.261/5.262 mm, action about
  19.1 s, measured joint commands 125.00 Hz, clock interval 1 ms, zero control
  deadline misses. Run 2 holds for 2 s with 1.71 micrometre drift.
- Artifacts and exact reproduction:
  `test_results/fr10_force_grounded_clock_20261001T111602Z/REPORT.md`, including
  `fixed_run_01` and `fixed_run_02_hold` summaries, traces and launch logs.
  Spring proxy evidence only; no robot hardware/contact was run. Both runs preserve
  a fixture-publisher SIGINT cleanup KeyboardInterrupt after successful insertion.

## 2026-10-01 — Matched insertion implementation: baseline and motion ownership

- Artifact root: `test_results/fr10_matched_insertion_20261001T115916Z/`.
  Captured dirty-tree status/diff, commit and 396 source/asset hashes before
  implementation with `python3 tools/fr10_matched_run_manifest.py <artifact>/baseline`.
  No reset, stash, `.orig` restoration or real-robot actuation.
- Baseline scoped build PASS. Functional CTest suites PASS for motion core,
  force core, insertion manager, pose controller (one intentional numerical-IK
  SKIP), description, bringup and swerve. FAIRINO pre-existing lint_cmake,
  uncrustify and remote-schema xmllint failures are retained in baseline logs.
- Existing standalone and dynamic mounted Gazebo JOINT/POSE smoke PASS, no
  ODE/LCP or server crash. Independently reproduced JOINT A → POSE B → JOINT A:
  baseline FAIL (last JOINT silently discarded); coordinator build PASS.
- Added ROS-independent `MotionCoordinator`: current-intent deduplication,
  command IDs/generations, worker-result ownership and bounded diagnostic events.
  Removed old per-mode cached-target admission logic. Added free-space-only
  `/fr10_motion/pose_target` and typed `/fr10_motion/command_status`.
- Implemented common source-time `TickClock` for JOINT/POSE/FORCE, physics-rate
  /clock for public Gazebo launch, scheduled reference-buffer future boundaries,
  generic bounded JOINT/POSE controlled-stop/replan, and publication commit for
  buffered references/Ruckig. No change to 1 degree lead or 50 ms feedback guard.
  New pure unit tests and existing pose suites PASS. Clock/handover Gazebo smoke
  PASS (including return-to-old-JOINT), no control deadline misses; traces retain
  four source scheduling gaps rather than hiding them.
- Reproduction: source `tools/source_simulation.sh`; scoped colcon build using
  `.colcon/rb_v3_humble/{build,install}`; CTest with writable `ROS_LOG_DIR`;
  `python3 tools/run_fr10_pose_gazebo_smoke.py <output> --domain 224 --port 11974
  --return-joint-target`. Harness uses isolated Gazebo, never cabinet hardware.

## 2026-10-01 — Matched CAD draft and nominal swept-fit gate

- Canonical supplied socket unchanged; SHA256
  `6f5bd5f58cf5583c3616ef552eec23280913977fb5555a5c7a9196aee039e7aa`.
  Added reproducible `tools/fr10_mating_geometry.py`: sections derive nine
  circular pin centres and minimum measured bore radii. Retained rear JLR
  body at its original scale; generated separate female collars, 45 degree
  lead-ins, recessed pilots and a seating shoulder.
- Draft mechanical simulation model lives in
  `vehicle_description/models/matched_socket_v2`; 364 convex collar cells,
  maximum radial facet deviation 0.092635 mm; explicit assumed total mass 3 kg,
  geometry-derived CoG and declared approximate box inertias. Opt-in
  `charging_tool_profile:=matched_socket_v2`; legacy remains default.
- `python3 tools/fr10_mating_fit_gate.py <artifact>/cad/swept_fit_fast.json`:
  nominal swept-surface fit PASS from -20 mm to 14.9 mm, no nose/body/shoulder
  surface interference; intentional shoulder contact at 15 mm + 50 um preload.
  Complete robot camera/mount collision remains a planning gate. This is CAD
  fit evidence, NOT force or physical-contact acceptance.
- Matched URDF expands/parses successfully, TCP derives from model datum;
  canonical mesh visual and collision remain same-sized. Multi-axis runtime,
  stationary-vehicle ownership, contact calibration and full scenario matrix
  are still IN PROGRESS; no matched force deployment PASS is claimed.

## 2026-10-01 — Matched FSM and dynamic vehicle parking gates

- Shared matched FSM unit gate PASS: edge alignment, clearance-gap traversal,
  seating, force/depth/dwell/actual-HOLD completion and no-progress stop. Coupled
  lateral norm 2 mm and total rotation-vector norm 2 degrees enforced. Restored
  legacy axial VERIFY nominal advancement after its regression caught the
  unintended change. Nominal and admittance state commit only on publication.
- Insertion CTest 6/6, force math/conditioning 2/2, swerve 4/4 PASS. Lease lives
  in the existing sole wheel actuator owner; replay cannot renew expiry, expiry
  remains latched, release cannot replay old velocity. The force session waits
  for fresh odometry and 200 ms source-time stationary dwell before staging;
  active pose drift/motion/lease loss latches a fault and requests controlled stop.
- Dynamic mounted Gazebo parking regression PASS at
  `test_results/fr10_matched_insertion_20261001T115916Z/motion/mounted_parking`:
  parked peak wheel-odometry speed 0.0000321 m/s, 29 drive requests rejected,
  expiry hold and release/new-command checks PASS; ODE LCP errors 0. This is NOT
  a mounted contact/insertion PASS or a ground-truth slip validation.
  Reproduce: `python3 tools/run_vehicle_parking_gazebo.py <output> --domain 229
  --port 11979` after sourcing `tools/source_simulation.sh`.
- Added dual inner/outer lead-ins sized to the measured walls (minimum 0.15 mm
  tip wall), rather than chamfering only the bore-facing outer edge. New nominal
  CAD swept fit PASS: `cad/swept_fit_dual_lead.json`.
- CoACD full socket decomposition exceeded 20,000 cells and is not accepted.
  Its generated cells were moved, recoverably, to
  `<artifact>/cad/rejected_convex_decomposition`; no supplied CAD was removed.
  For stationary Gazebo/FCL socket use exact original triangle surfaces,
  partitioned at z=18 mm into contact and protected rear, with no artificial
  cap or filled hole. Surface-area discrepancy 1.69e-11 m². ODE implementation
  evidence: https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEMesh.cc#L130
  MuJoCo convex collision is a separate pending gate, not claimed equivalent.
- Added six deterministic station candidates and four start configurations,
  with face-direction rules and model-derived mount transforms. Their
  reachability/collision/contact acceptance remains PENDING. Matched profile
  stays disabled and opt-in via `station:=front_mid`; multi-axis runtime is
  fail-closed until the full envelope planner returns validation. Demo remains
  gated. Legacy force launch defaults and native hardware contact lock retained.

## 2026-10-02 — Terminal reference commit and matched free-POSE gate

- Fixed the reproducible matched free-POSE rejection after controlled braking:
  HOLD and post-stop planning were observable before publication of the final
  zero-qdot/qddot reference. Executor now proposes terminal state and commits
  HOLD only with the acknowledged command. Runtime handover/coordinator consume
  the committed state, preserving the original joint limits and feedback guard.
  Deterministic reference-transaction test covers the unacknowledged final
  braking sample and the exact stationary replan boundary.
- Scoped build 14 packages PASS; core 5/5, pose 70/70 (one existing deliberate
  numerical IK SKIP), insertion 7/7 and bringup 8/8 CTest PASS. Test a freshly
  installed ABI, not an old overlay library after a build-directory-only relink.
- Matched standalone free POSE -> CAD preinsert PASS at
  `test_results/fr10_matched_insertion_20261001T115916Z/gazebo/standalone_preinsert_5`.
  Known 3 kg raw wrench norm 29.4173 N vs expected 29.41995 N; exact canonical
  CAD objects acknowledged by PlanningScene. Source timing 124.890 Hz,
  max tick 0.077 ms, zero tick deadline misses, two source scheduling gaps,
  zero LCP errors. Explicit controlled-stop/HOLD confirmed before physics exit.
  A separate user-owned Gazebo instance was left untouched; these measurements
  do not constitute isolated final timing/contact/self-alignment acceptance.
- Reproduce after sourcing `tools/source_simulation.sh`:
  `python3 tools/run_matched_station_gazebo.py <new_output> --domain 190 --port 11984`.
  Failed domain 233 run retained in `standalone_preinsert_4`; test harness now
  rejects out-of-range Fast DDS domains before launching anything.
- Final current model: shoulder datum z=24 mm, TCP z=39 mm at sensor frame;
  final nominal swept gate `cad/swept_fit_final.json` PASS. Contact stiffness
  20,000 N/m and damping 100 Ns/m are declared simulation assumptions awaiting
  measured calibration, not vendor/material data. Multi-axis profile remains
  disabled. Shutdown station publisher's second SIGINT is being fixed; no
  matched physical-contact PASS or mounted insertion PASS is claimed.
- Additional gravity convention defect found by review: source sign was applied
  only to raw load, not modeled gravity. Native gravity is now subtracted before
  converting to tool-on-environment convention and canonical tare; negative-sign
  source regression added. This change still requires rebuilt unit/calibration gates.

## 2026-10-02 — Unified insertion end-to-end implementation

- User authorized implementation and iterative feedback-driven testing across Gazebo, MuJoCo and Docker. Pose is the socket mouth; matched CAD simulation profile and automatic completion to HOLD. Prefer compatible Fairino 3.9.3 image.
- Baseline source hashes and existing diff saved under `test_results/fr10_e2e_20261002`. Work is IN_PROGRESS; no contact acceptance claimed.

### Current feedback (implementation continues)

- Added automatic mouth-pose admission, shared action/topic results, branch screening for the full insertion corridor, OMPL joint detours and asynchronous online collision/braking validation. Core stays ROS/MoveIt/SDK free; ServoJ remains the native eight-parameter 8 ms call.
- Fixed nominal lead after persistent admittance retreat, HOLD ownership, explicit recovery retract and typed planning rejection. Build 12 passed; core 5/5, planning 1/1, insertion 7/7, pose 70/70 (one configured skip), bringup 8/8 passed. Bringup required a writable ROS_LOG_DIR; initial sandbox log-path failures were infrastructure, then rerun passed.
- Canonical CAD swept fit passed 141 poses. MuJoCo preserves CAD holes using 50 um backed convex surface patches; this approximation requires independently measured penetration below 50 um and does not replace complete physical rear/self-contact acceptance.
- Gazebo tests 03/04 confirmed torque PID on all six joints, exposed incorrect integral torque clamp and a j1 limit cycle; calibration is still in progress. Earlier dynamic_servo_01/02 did not actually activate PID because the combined ros2_control Xacro omitted the parameters.
- MuJoCo 05 reached the seat but stopped on raw force limit. MuJoCo 06 reached VERIFY_INSERTION, then stopped on lost contact. Neither is an insertion PASS. Contact stiffness, seating speed and keyed-pose restoring stiffness are under feedback-driven calibration; safety limits have not been relaxed.
- Docker migrated to manufacturer 3.9.3 image 7f4124d7fcfa95a604802bff20c1ae0f65a18b5d2e782cb88f49921dc63daab2 at 192.168.58.2. Prior 3.7.8 container retained as fairino-container-v378-backup-20261002, disconnected/stopped. Native read-only SDK transport passes; actual initial joints [0,-90,90,-90,-90,0] deg. Cabinet is Manual; explicit user choice/authorization for Auto is pending, consistent with AGENTS.md. No Mode/enable/alarm-reset call has been issued.
- Evidence directory: test_results/fr10_e2e_20261002. Status remains IN_PROGRESS; full contact/HOLD, multi-station and native motion acceptance remain outstanding.

### Further commissioning feedback — 2026-10-02

- New measured-velocity damping GazeboTorqueSystem avoids derivative kicks from ODE joint-angle noise. Independent Gazebo link pose exposed transverse shoulder hinge tilt under the 100-iteration QuickStep solver; ODE's quaternion hinge-angle calculation can report that tilt as j1 rotation. Exact/world solver reached the seat but generated 90 LCP errors and is rejected. QuickStep 2000 iterations / SOR 1.0 has zero LCP errors in run 10; physical TCP/FK consistency remains an acceptance gate.
- Separated fresh online proposal deadlines from accepted command leases, preserving wrench freshness and controlled stop on expiry. Added deterministic latency/expiry tests. HOLD retains ownership; unverified intermittent seat contact may reseat under unchanged safety/depth/no-progress guards, while lost contact after accepted HOLD still faults.
- Gazebo 09 and MuJoCo 09/10 physically reached the matched seat but stopped on NO_PROGRESS; none is a contact/HOLD PASS. Seating force convergence now counts as progress, without phase cycling or oscillation renewing the watchdog. Axial speed reduction alone did not stabilize MuJoCo lateral/rotational compliance; run 11 evaluates higher restoring stiffness/damping and bounded correction speeds.
- Four actual initial configurations are now selected by initial_state / --initial-state in Gazebo and MuJoCo. Source/binary manifest includes the torque plugin, external Gazebo observer and generated physics models. Physical TCP and all-world Gazebo contact evidence are separate from controller feedback. Test charging_station regression is fixed and bringup 8/8 passes.
- Build 19 passed 15 packages. Build 20 adds full sampled online Ruckig transition screening alongside live q/v/a braking on the worker. This screens the captured boundary; it does not certify every possible six-dimensional correction or a different future activation boundary.
- Docker rechecked read-only: compatible SDK/image, healthy and enabled, still Manual (mode 1). No Mode/enable/reset command issued. Native motion waits for the previously requested operator Auto selection or explicit simulator-only authorization; unaffected simulation work continues.

### Timing and exact contact model — 2026-10-03

- Build 27: 15 packages PASS. Planning 2/2 and core 5/5 CTest PASS; Python planning-facet/model/bridge/evidence/native gates 22/22 PASS. Complete current CTest handoff is still pending.
- Fixed late SUCCESS proposal consumption without falsely reporting backend rejection; pending sequence must match exactly. Accepted online command lease is 64 ms, pending proposal deadline remains 48 ms; independent wrench expiry is still 50 ms. Full transition and captured q/v/a braking remain worker validated.
- MuJoCo 14 failed in free space on a pending deadline; collision checks took 35–38 ms. FCL now concatenates canonical facets of 364 nose cells into one BVH without removing any triangle. Python facet equivalence/tamper tests 6/6 PASS. Source/binary hashes and live planning model are captured separately from physical URDF. Worker timing improved to approximately 29–31 ms; MuJoCo clock rounds to integer nanoseconds to remove accumulated 1 ns deadline jitter.
- MuJoCo 15 completed 3,200+ collision-checked proposals and reached 15 mm physical depth, keyed fit under 16 um, zero unexpected contacts, maximum penetration 2.07 um. It still failed NO_PROGRESS because seating force did not stay within verification force/rate limits. Contact/HOLD remains unaccepted; no safety or verification threshold was weakened.
- MuJoCo socket now uses fully rigid zero-radius triangle flexes from original contact/rear CAD. This replaces 11,102 backed overlapping convex shells; no added DOFs or filled bores. New deterministic tests compare every compiled triangle to CAD and verify open-bores/actual seating load (2/2 PASS). MuJoCo 16 and Gazebo 13 are evaluating dynamic contact/HOLD; source manifests record this change.
- Docker fresh read-only preflight after build 27: native SDK transport healthy, image/IP identity correct, still Manual. Native actuation has not started; previously requested Auto choice remains pending. Verified native gate blocks before actuator launch in Manual; virtual contact will be explicitly identified as a separate CAD plant following SDK joints.

### End-to-end and planner feedback — 2026-10-03

- Gazebo contact run 14 is the first complete one-pose contact/HOLD PASS: physical depth 15.071 mm, keyed feature error 150.7 um, stationary physical TCP/FK difference 13.5 um, zero unexpected world contacts and zero LCP errors; shutdown controlled stop acknowledged. It used the earlier damping profile, so does not accept subsequent gain changes or other station/start combinations.
- MuJoCo 16/17 reached the seat but failed NO_PROGRESS. Constant-reference CAD physics diagnostic at the seat was stable (6.818 N mean, 0.007 N standard deviation). Continuing micro corrections, rather than free-space reachability, remain a convergence issue. A continuous net-drive dead zone was added without filtering or changing safety/verification measurements; deterministic six-axis equilibrium, live-velocity damping and invalid-config tests pass.
- MuJoCo 18 stopped on a pending worker deadline during concurrent compilation/matrix work. MuJoCo 19 reached the seat, then stopped on raw FORCE_LIMIT following a large numerical contact impulse; maximum penetration 593 um fails the 50 um gate. Neither is contact/HOLD accepted. MuJoCo 20 evaluates a larger bounded drive dead zone and an explicit post-HOLD retract; safety/verification thresholds remain unchanged.
- Build 30 passed 15 packages. Core 5/5, planning 2/2, force 2/2, insertion 7/7, pose 70/70 (one configured numerical IK skip), torque plugin 1/1, bringup 8/8 and vehicle-description 2/2 CTest pass. Driver CTest exposes legacy formatting/CMake lint; supported plugin formatting and explicit exclusion of unchanged unbuilt manufacturer sources are being addressed. No SDK ABI/call changes are intended.
- Immutable collision checks now privately cache only clear link pairs with fixed relative transforms and >=1 mm measured distance. A 50-state comparison to complete collision checks, fixed-overlap rejection, moving vehicle/world checks and scene ACM preservation pass. Canonical model has 71 such pairs; online measured timing still approximately 25–30 ms, so no stronger performance claim is made.
- Shared sampled controlled-stop validation replaces duplicated corridor code and makes the read-only probe match runtime braking screening. RETRACT now keeps the raw force safety monitor active; overload-during-retract regression passes. The harness separately records seated and retracted physical evidence; withdrawal cannot overwrite the seated HOLD window. New runs archive actual text sources/configuration in source_snapshot.zip alongside CAD/binary hashes.
- Planning matrix 01 passed 19/24 real-model station/start cases. Matrix 02 found an RRT detour from old folded/front_mid, then hit the external 60 s probe cap during trajectory processing. Improved phase evidence and per-case timeout handling are in progress; this timeout is not reported as a collision-planning failure. Two old folded presets begin in actual collision for elevated stations and require an explicitly corrected commissioning start catalog.
- Native Docker remains operator-mode blocked; no Mode, enable or alarm reset was sent. End-to-end status remains IN_PROGRESS until current-profile simulation/retract and station/start acceptance are completed.

### Reachability and time-law acceptance — 2026-10-03

- Build 32 passed 15 packages; complete current CTest across core, planning, force, insertion, pose, torque, bringup, description and supported driver passed (one configured numerical-IK skip). Manufacturer SDK and unbuilt legacy command-server/example sources retain their original formatting; CMake/runtime plugin/header style checks pass. Supported plugin C++ token equivalence after line-splice/comment processing was verified before rebuilding.
- Planning matrix 03 passed 22/24 with the original folded start. Both failures are actual initial collisions: elevated support column/wrist1 37.949 mm overlap; tilted socket/wrist1 0.269 mm overlap. Those are refused, not automatically escaped from an unvalidated colliding initial state. The catalog folded preset was explicitly changed to [-120,120,-90,-90] deg for j2-j5; it initializes simulation only, never overwrites live joints.
- Joint routes now use uniform six-joint arc sampling and collision-checked corner fillets; a colliding blend is discarded. Fixed endpoint and subsequent timed-spline collision checks remain mandatory. Obstacle/blend fallback/uniform sampling tests and 50-repeat stochastic route tests pass.
- FREE_SPACE time parameterization now evaluates Cartesian derivatives on the actual smooth analytical FK of the joint spline, avoiding artificial jerk jumps from piecewise TCP chords. INSERTION/retract retain their strict Cartesian reference. Deterministic nonlinear-FK speed/reference, forbidden insertion callback and FK-failure rejection tests pass. Independent MoveIt FK/collision still validates every final timed sample outside the tick.
- Matrix 04 passed 24/24 catalog station/start cases. Maximum route time 1.908 s; maximum trajectory duration 7.704 s. Separate old folded/front_mid trial still finds a collision/singularity-free RRT route and fully validates a 12.408 s trajectory; previous trial with chord-based timing required 129.016 s (different stochastic route, not a matched speed benchmark). Source/config archive and binary hashes are saved per run.
- MuJoCo 20 passed independent physical seated fit (maximum penetration 1.032 um), but failed NO_PROGRESS: 5 Hz conditioned load/rate had only 80 ms continuous acceptable dwell. Replay at 1 Hz yields 488 ms, with the filter time constant below the calibrated compliance/contact response. The current commissioning overlay uses 1 Hz; raw overload/force/torque and all verification thresholds remain unchanged.
- MuJoCo 21 failed in free space with a fresh, worker-validated target rejected by the executor. Runtime admits targets before updating executor feedback, so an accepted lease based on previous-tick feedback can be shortened by clock/sample batching. Build 33 adds explicit bounded current-source admission time, preserving the original proposal freshness test and full 64 ms accepted lease; deterministic past/future/nonfinite admission and expiry tests are being verified. No contact/HOLD claim is made for the current force profile yet.

### Current-profile end-to-end acceptance — 2026-10-03

- Build 33 passed 15 packages; core 5/5, planning 2/2, insertion 7/7 and pose 70/70 (one configured skip) passed. MuJoCo contact run 22 passed one socket pose through preinsert, force insertion, verified HOLD and explicit retract. Independent depth 14.998 mm, keyed error <=23.18 um, HOLD drift 0.88 um, peak penetration 1.14 um, zero unexpected fixture contacts; retract at -30 mm with 1.48 um depth error, unloaded force owner released, shutdown controlled stop acknowledged.
- Gazebo run 15 failed to reach preinsert within the goal timeout; accepted paths were slowed to 141/414 s by angular jerk despite nominal constant orientation. It also lacked independent transport data. Shutdown controlled stop was acknowledged; no contact acceptance is claimed. Investigation found precision loss from acos quaternion angular distance at tiny rotations and an observer/master startup race. Build 34 evaluates atan2 relative-quaternion distance with small-angle/live-seed regressions; the harness now starts its read-only Gazebo observer after runtime readiness and requires a physical pose sample before sending motion.
- Fresh Docker read-only preflight after build 33 still reports healthy native transport, controller/image 3.9.3 and Manual mode 1. No Mode, enable or reset command has been sent; native motion is still pending the existing operator Auto/explicit simulator-only authorization choice.

- Build 34 passed 15 packages and all CTest after driver schema network retry. Gazebo 16 confirmed observer startup/readiness but stopped on the unchanged 1 degree reference lead guard when a 0.968 s free-space approach exceeded plant tracking. It was explicitly stopped/canceled for diagnosis; no PASS. Build 35 passed 15 packages and complete CTest; live small transverse boundary velocity now requires controlled stop without an absolute residual allowance, and latched motion faults terminate staging through a typed force result instead of waiting for the goal timeout.
- Gazebo 17 reached preinsert with a 4.448 s approach, then successful contact/HOLD: independent depth 15.042 mm, keyed error 89.96 um, stationary physical FK error 5.46 um, reported HOLD drift 0.53 um, no unexpected contacts/LCP errors. Explicit retract/result/unload gates passed but peak physical penetration during withdrawal was 1.637 mm, so overall contact acceptance FAIL. A temporary feedback pause was investigated with attached read-only stacks; those showed framework wait rather than an application mutex cycle, and do not prove an application deadlock. Build 36 reduces strict withdrawal from 15 to 3 mm/s and requires keyed clearance while retracting inside the socket. Harness HOLD observation increases from 1 to 5 s and records independent physical drift. Revalidation remains in progress.
- Build 36 passed 15 packages, complete CTest and independent evidence tests 4/4. Gazebo 18 passed seating/HOLD over 5 s (15.045 mm maximum depth, keyed error <=90.80 um), then failed the new withdrawal clearance guard with typed TRACKING_FAILURE/controlled stop at 14.923 mm. Peak penetration 539.75 um still exceeds its 500 um gate, so overall FAIL. Slower reference alone does not cure the load-release transient; no guard was weakened.
- Build 37 replaces Gazebo gravity-holding PID integrals with world-frame gravitational moments from cached physical descendant-link mass/CoG, summed about each actual joint axis/anchor. Canonical gains use zero integration, stronger wrist proportional stiffness and measured-velocity damping. URDF effort clamps and torque-only actuation remain unchanged. Deterministic gravity sign/lever translation, exact unloaded equilibrium, external-load release without integral storage, feedforward effort saturation and invalid input tests pass; complete CTest passes. Dynamic physical acceptance remains pending.
- Read-only Docker preflight after build 36 again confirms correct 3.9.3 image/SDK, healthy and enabled, Manual mode 1; native motion remains blocked by the pending operator-mode choice. Manufacturer manual also contains a ServoJ inconsistency: stated cmdT 1–1.6 ms, adjacent example 8 ms. The pipeline documents both and preserves the repository's verified eight-parameter 8 ms call.

### Physics-step servo and raw impulse retention — 2026-10-03

- Gazebo 19 reached a physically fitted seat (15.033 mm, keyed error <=160.37 um, penetration <=49.93 um) but failed STALE_WRENCH before verified force HOLD. Physics slowed below real time: 125 Hz simulation-clock sensing could exceed the unchanged 50 ms wall receipt guard. Run 20 with stronger shared gains failed planning after initial wrist drift; neither run is accepted.
- Raw FT sensing now samples at 1000 Hz simulation time; the 125 Hz insertion consumer retains signed component extrema and invalid packets between ticks. Conditioning/tare still use the latest individual sample, and duplicate timestamps cannot renew freshness. Deterministic short-impulse overload, torque, NaN latching and reset tests pass; no force/torque/freshness threshold changed.
- Gazebo torque computation moved from the 8 ms ros2_control write callback to every 1 ms physics update. The command interface remains six position targets, applied through effort-limited torques only. MuJoCo and Gazebo load the same canonical zero-integral PD gains; fresh physical verification remains pending. The harness refuses motion if its initial independent known-load check fails.
- Build 39 completed all 15 packages. Full CTest is running, including the updated 1000 Hz sensor contract. Latest accepted full contact/retract evidence remains MuJoCo 22 on the earlier timing build; current Gazebo withdrawal and mounted behavior remain outstanding.
- Build 39 full CTest passed all nine packages. Gazebo 21 reproduced low-inertia wrist oscillation before motion (raw force standard deviation 54.21 N); mean-load calibration alone missed it. The harness now also requires stable idle force/torque before admitting a pose. Run 21 is FAIL, with controlled stop acknowledged.
- Build 40 evaluates coupled stable PD with the complete six-joint physical mass matrix assembled from actual descendant link inertia and Jacobians; implicit damping replaces the unstable explicit update. It still applies only URDF-clamped torque, cancels modeled gravity, and preserves all unmodeled physical contacts. Coupled low-inertia wrist/load-release and invalid-inertia/period regressions were added. Fresh contact/retract acceptance remains pending.
- Fresh Docker read-only preflight after build 40 again passes transport/version/image identity, remains Manual mode 1, and has not actuated or changed mode.
- Build 40 completed all 15 packages; complete CTest across all nine packages passed, including coupled stable-PD regressions. Independent evidence/native-gate Python tests passed 14/14. Gazebo 22 passes idle stability and reaches the actual seat after the 4.448 s preinsert path; force convergence/HOLD/withdrawal are still being measured. Gazebo CAD collision slows source-clock progress below real time; unchanged unique-wrench wall freshness remains satisfied by oversampling.
- Gazebo 22 now PASS one-pose contact/HOLD and explicit retract with current build/profile. Independent seated depth 15.038–15.039 mm, keyed error <=71.46 um, physical HOLD drift 1.23 um, maximum physical penetration 35.56 um across the complete run, zero unexpected station/world contacts, zero LCP. Retract independent depth error 4.47 um; stationary physical TCP/FK difference 18.06 um; final unload/owner-release and shutdown controlled-stop gates pass. This is front_mid/home arm-only acceptance, not a claim for all stations or mounted physics.
- MuJoCo 23 (tilted_up/folded) reached preinsert but failed COMMAND_REJECTED (safety 16) at -28.516 mm before contact: a collision-validated candidate arrived at the 48 ms boundary while the executor was already STOPPING. It is retained as FAIL; stop acknowledged. Build 41 captures one admission clock consistently and exposes the actual accepted lease deadline in status, with transition diagnostics, to distinguish source scheduling expiry from IK/collision rejection. Build 41 all 15 packages and complete CTest pass; MuJoCo 24 revalidation is running.
- Build configuration audit found the Gazebo servo compiled with no optimization and persisted CMake dependency directories pointing at the previous root install, despite runtime resolving the canonical current libraries. The build script now requests Release and clears only workspace-package cached *_DIR values so colcon resolves them from its selected runtime. This is a reproducibility/performance correction, not evidence that stale linking caused run 23. Fresh build and acceptance remain pending.
- MuJoCo 24 (tilted_up/folded) now PASS one pose through contact/HOLD and explicit retract on build 41. Independent depth 14.995–14.996 mm, keyed error <=11.84 um, physical HOLD drift 1.37 um, penetration <=0.88 um, zero unexpected fixture contact; retract depth error 1.04 um, unload/owner-release and controlled shutdown pass. No online target rejection occurred in this run. This accepts the tilted commissioning case but does not erase run 23 or certify host scheduling under load. Build 42 applies clean dependency resolution/Release before the next mounted and sensor-fault trials.
- Build 42 completed all 15 packages and complete CTest across nine packages PASS. Cached workspace dependency directories now resolve to `.colcon/rb_v3_humble/install`; Gazebo servo compiler flags include `-O3 -DNDEBUG`. Read-only real-model matrix 05 passes all 24 station/start cases on this runtime, including sampled timed FK/collision/corridor/braking validation. Mounted Gazebo physical trial is running; native actuation remains operator-mode blocked.
- Mounted Gazebo 01 failed TRACKING_FAILURE before contact at -27.176 mm, controlled stop acknowledged. Independent base observation was missing because Gazebo lumps the fixed arm mount into `base_footprint`. The observer now measures that actual physical root and composes the canonical URDF fixed chain to recover `fr10_base_link`; rotated/translated mount and partial-live-record regression tests pass 5/5. Build 43 complete CTest passes; base interlock fault codes are now logged without changing thresholds.
- Mounted 02 failed during initial feedback startup before admitting motion. Build 44 moves the existing first-complete-JointState wait ahead of simulation resume timeout handling. A new isolated ROS transport regression publishes clock first, delays initial mock joint data for 250 ms (configured timeout is 50 ms), verifies readiness after data arrives, then verifies COMM_FAILURE on initialized feedback dropout. It passes; no actuator/SDK process is launched by this test. Build 45 registers this meaningful positive/negative startup test in CTest. Mounted acceptance and physical sensor-fault evidence still remain outstanding.

### Mounted physical frame and parking diagnosis — 2026-10-03

- Mounted 03 and 04 stop before contact with base interlock DRIFT (enum 5), not lease loss. Run 04 records both wheel odometry and Gazebo transport: physical arm-base displacement about 23.2 mm while wheel odometry reports about 0.113 mm. No guard was weakened and neither run is accepted.
- Gazebo Classic World::ProcessMessages publishes model and link RelativePose. The independent observer now composes all parent model transforms from the same source message into world, rejects missing parents/nonfinite poses, labels schema 2/world, and differentiates velocity in source time. Deterministic nested/nonidentity/missing-parent/invalid-pose checks PASS. Earlier arm-only measurements have identity parent transforms; previous mounted world measurements were incomplete and do not establish acceptance.
- Build 46 evaluates static-friction wheel contact in the precision profile (navigation retains its prior slip), full physical Gazebo P3D base feedback with strict expected world/base_footprint frames, and a single physical root TF. Socket CAD and frames stay fixed in world; planning snapshots and the test input pose are transformed into the measured arm frame. The floating vehicle is not fixed/teleported to world. Current physical mounted acceptance remains pending.
- Build 46: all 15 packages and complete CTest PASS. Mounted 05 correctly rejects admission before arm actuation: wheel odometry reports near-zero motion while the direct-joint GazeboSystem plant reports inconsistent physical velocities (about -0.08 m/s vertical with nearly fixed height). Zero tangential slip alone does not fix this and run 05 remains FAIL. Build 47 evaluates eight effort-limited ODE joint motors for the precision profile, replacing direct steering position/drive velocity assignment without constraining the chassis to world.
- Build 47 all 15 packages and complete CTest PASS. Physical ODE motors reduce base displacement to micrometres in mounted 06, but the P3D velocity fields remain inconsistent with source-pose derivatives and correctly prevent admission. A preliminary selected CTest invocation failed only because ROS_LOG_DIR defaulted outside the writable workspace; the corrected complete run uses /tmp and passes.
- Build 48 all 15 packages and complete CTest PASS. Full physical base pose is the precision localization authority; unfiltered source-pose twist is computed in the child frame by shared quaternion math, with duplicate, interval-gap, antipodal, invalid-pose and one-sample movement regressions. Raw P3D fields are retained independently in physical_base_odometry.json. Mounted 07 now admits parking but stops safely during free-space staging (MOVING enum 4): a small actual chassis reaction exceeds the unchanged 100 um/s / 100 urad/s velocity barriers. It is FAIL. Re-timing the mounted free-space approach is the next correction; no force/contact guard is relaxed.

- Config revision 49 caps mounted free-space acceleration/jerk at 0.10/0.025 without changing contact/parking barriers. Mounted 08 fails before command on disconnected world/base TF; mounted 09 still fails MOVING while the executor brakes before path activation. Neither accepts these caps. Subsequent diagnosis identifies two independent causes: the swerve spawner overrides the composed precision YAML (TF stays ON), and ODE raw joint velocities disagree with actual source-time encoder displacement before any command.
- Mounted 10 fails only on the probe's unwritable default ROS log directory; mounted 11 fails sandbox socket setup. No motion acceptance is claimed. Mounted 12 uses a private DDS domain/port, the corrected shared spawner parameter file and one existing TF listener: runtime TF is OFF, frame warnings disappear, but false initial joint velocity still causes braking and MOVING. In the 1.8 s pre-command window raw joint rates reach 0.099 rad/s while encoder source derivatives remain below 1.11e-5 rad/s. Full measured q/v/source feedback is now archived even after failure.
- Build 50 evaluates source-time encoder velocity for the Gazebo ROS feedback only; the physics-step stable PD still uses actual raw ODE damping velocity and unchanged URDF effort limits. Deterministic stationary/ramp/nonzero-motion/duplicate/backward/gap/NaN checks are added; no moving-boundary or force guard is weakened. A new launch regression checks that the generated publish_tf=false YAML actually reaches the child spawner's -p argument. Physical revalidation remains pending.
- Build 50 passes all 15 packages and complete CTest across ten packages, including swerve. Mounted 13 eliminates the false RETARGET_TIMING stop and activates a valid trajectory, but remains FAIL/MOVING: the first actual reference differs from the old URDF-held servo target by ~0.001 rad, although it agrees with measured encoder within 1 umrad. Biased raw ODE damping produces this static offset; changing to the measured reference causes a real 0.30 mrad/s chassis reaction. Independent source traces prove it; thresholds are unchanged.
- Build 51 uses a separate physics-step encoder estimator for stable-PD damping, so the servo and ROS feedback observe actual q displacement at their respective 1 ms/8 ms source periods. No initial dummy trajectory, automatic motion enable, target offset, chassis constraint, gain/effort change or guard weakening is introduced. Coupled dynamic convergence with encoder sampling is tested. All 15 packages build and complete CTest across ten packages PASS (one configured numerical IK skip). Mounted 14 physical revalidation is running. Fresh native preflight after 50 remains healthy Manual 1; no native actuation/mode/enable/reset command has been issued.
- Mounted 14 preserves the initial datum to ~4.4e-8 rad and activates 108 samples, but correctly fails MOVING when real approach velocity reaches 0.116 rad/s. Config revision 52 caps mounted q velocity/acceleration/jerk at 0.025/0.025/0.003125 and linear scale at 0.025. Vehicle CTest 9/9 PASS. Mounted 15 passes parking and preinsert, completes 3,105 checked force proposals and enters the socket, then fails initialized feedback freshness: unique joints are 8 ms apart in source time but one wall receive interval is 52.246 ms. COMM_FAILURE is retained, with unconfirmed stop; no contact/HOLD acceptance is claimed. Its launcher was explicitly stopped after the motion node exited, and failure evidence was saved.
- Config revision 53 reads/publishes actual Gazebo joint state each physics step (1 kHz), while executor/command stream remains 125 Hz and the 50 ms wall freshness barrier stays unchanged. This mirrors the already oversampled physical F/T/base feeds without fabricating samples. Shared spawner-config regression now checks the 1 kHz manager setting too; vehicle CTest 9/9 PASS. Harness detects loss of its motion node rather than waiting for a surviving launcher until the outer deadline. Mounted 16 complete physical revalidation is running.
- Mounted 16 reaches 14.939 mm and real shoulder contact with 4,361 checked proposals, no initialized feedback timeout and no earlier parking violation. It correctly fails MOVING during SEATING: source-pose pitch velocity peaks at 159.5 urad/s near first shoulder contact. Physical penetration is only 3.76 um and stationary FK/physical difference <=31.18 um, but this is not a HOLD/contact PASS. Controlled stop acknowledged. RuntimeProbe's subscriber receive-gap metrics are observer timing, not the backend's callback timing.
- Build 54 adds a common C1 depth-based nominal speed taper before the shoulder, using existing reference lead, correction velocity and acceleration. No force, parking or verification threshold changes. Positive floor/bounded monotonic speed/zero-gradient joins/invalid bounds and existing transaction rollback tests PASS. All 15 packages build and complete CTest across ten packages PASS. Fixed the actual controller parameter name linear_scale (earlier linear_velocity_scale override was undeclared and ineffective); the launch regression and harness now verify/capture effective motion parameters. DDS traffic is restricted to localhost using ROS_LOCALHOST_ONLY. Mounted 17 runs with measured effective executor125Hz/timeout50ms/qv0.025/qa0.025/qj0.003125/linear0.025; physical full acceptance is pending.

- Mounted 17 reaches VERIFY and physical seating but fails MOVING (pitch 118.46 urad/s); never accepted as HOLD. Revision 55 reduces the shared axial correction speed to 50 um/s. Mounted 18 fails STALE before contact; default ROS callback serialization is investigated.
- Build 57 gives parking feedback its own callback group and latest-sample QoS. A no-actuator transport regression survives an unrelated 80 ms callback and rejects a subsequently stale source. All 15 packages build and complete CTest across ten packages PASS. Mounted 19 has healthy feedback (backend age <=7.9 ms) and zero tick deadlines, but fails VERIFY/MOVING at pitch 104.32 urad/s. Independent physical fit passes; action/HOLD does not.
- Solver experiment 58 increases QuickStep iterations from 2000 to 4000; vehicle CTest 9/9 PASS, mounted 20 FAIL/MOVING at only 3.634 mm depth. Independent world coordinates show 0.33 mm lateral error and actual pin/nose side contacts. The frozen initial arm-base-frame goal drifts with the floating mounted base's changing attitude. Solver change is reverted. Next correction anchors the one input pose and fixture geometry in their fixed task frame and revalidates alignment at preinsert; no fixture teleport or parking-threshold relaxation.
- Harness shutdown now targets only a verified descendant motion-node PID from its own launch log, checks the canonical executable, and lets its final brake finish while physics/feedback remain alive before stopping launchers. Final pass additionally requires this ordering and no shutdown-braking deadline error.
- Build 59 catches a missing shared-math include before runtime. Build 60 all 15 packages and complete CTest across ten packages PASS. New deterministic checks cover rigid task-frame round-trip, base tilt changing only the arm-relative goal, no force integration at the approach reframe, original timeout retention and rejection after approach ownership ends. Mounted input is anchored at its measurement timestamp; goal and original world-frame collision geometry are resolved together on the worker, with fresh measured TF. One strict preinsert refinement reuses the existing pipeline before tare. The floating base and original parking anchor/guards remain unchanged. Mounted 21 full physical validation follows.
- Mounted 21 Build 60 FAIL/STALE while completing the first preinsert path (-30.357 mm); no physical contact claimed. Feedback/controlled stop confirmed. Verified owned-node-first shutdown completes with zero braking deadline errors. Build 61 isolates parking feedback and lease heartbeat in an owned SingleThreadedExecutor (no actuator), not merely another callback group. Its regression saturates all three ordinary executor workers with 80 ms unrelated callbacks and still requires fresh unique feedback, then requires stale-source rejection. Original 50 ms/velocity/drift barriers unchanged. Task-frame cache now drops partial/frame-changing updates and revisions the scene, preventing old fixed-frame geometry reuse.
- Build 61 all 15 packages and full CTest across ten packages PASS; executor-saturation/stale-source regression passes. Mounted 22 is running in its own DDS domain/physics port. No native mode/enable/reset/motion commands have been sent.
- Mounted 22 Build 61 has healthy isolated parking transport through the entire insertion and successful stationary task-frame refinement (0.408 s strict correction). Physical keyed fit <=99.5 um, FK/physics <=24.65 um and penetration 2.251 um; no actuation/timing timeout. It remains FAIL/SEATING MOVING at depth14.929 mm, pitch122.51 urad/s. Physical contact depth is ~14.997 mm: allowed base/plant displacement makes the shoulder ~70 um earlier than FK-derived nominal depth. Original C1 taper ignores accepted depth uncertainty and arrives too fast. Build 62 includes existing depth_tolerance_m in the shared taper's braking margin, with unchanged force gains/speed caps/parking barriers and no new FSM cases. Controlled stop and motion-node-first shutdown confirmed.
- Build 62 all 15 packages/full CTest across ten packages PASS; uncertainty-aware taper test and invalid-rotation/no-mutation reframe check added. Mounted 23 fails LEASE_LOST at depth0.830 mm before seating, so taper physical acceptance remains pending. Independent parking trace retains STATIONARY/same owner/increasing sequence at ~33 ms wall cadence. Found concurrent observation ordering defect: caller captures now before a callback can publish a newer receipt, then the adapter reads that newer snapshot and classifies its receipt as future. Build 63 captures steady time after reading both snapshots; freshFeedback follows the same contract. No freshness threshold is changed. Transport regression now includes an acknowledged synthetic lease and repeatedly requires the active monitor to stay ready while all ordinary workers are occupied, then rejects a stopped source. Added actual status/base receipt-age fault diagnostics.
- Build 63 all 15 packages/full CTest across ten packages PASS. Acknowledged-lease + fresh-source monitor stays ready with all three ordinary executor workers blocked, then rejects source loss; no actuator is present in this regression. MuJoCo HOLD fault harness now requires independent physical fit before injection and checks motion-node liveness while awaiting the fault result. Mounted 24 complete physical validation is running.
- Mounted 24 Build 63 FAIL/SEATING MOVING at depth14.930 mm, pitch121.59 urad/s; actual status age21.291 ms/base age0.268 ms confirm transport/lease are healthy. Controlled stop and shutdown PASS, no physical insertion/HOLD acceptance. Independent URDF FK recomputed from 1 kHz encoder/source data and same-source physical base transform matches world TCP to ~2 um before contact, ~6.4 um during rebound (contact_kinematics.json). This corrects the earlier 70 um early-shoulder inference: the tool physically rebounds on contact; stale/cross-time comparison overstated frame/depth mismatch. The wider C1 margin remains a valid uncertainty bound, but near-contact velocity is dominated by the existing nominal floor plus force correction. Config revision64 lowers only the common axial correction cap/floor to25 um/s; original parking/force/verification limits, gains, CAD and solver2000 remain unchanged. Harness now records live axial velocity array/depth tolerance/lead/task frame too. Fresh MuJoCo tilted/folded HOLD + overload-stop validation follows before the next mounted run.
- MuJoCo25 revision64 tilted_up/folded reaches real seat and verified 5-wall-second HOLD: depth14.995957–14.996779 mm, keyed fit11.894 um, physical drift1.073 um, penetration0.693 um, zero unexpected contacts. However the full fault test FAILS: overload at already-stationary HOLD enters FAILED immediately (stop_complete is already true), while completion_reported still suppresses a second terminal result. This is not fault acceptance. Build64 adds a no-actuator ROS lifecycle regression: success once, stable HOLD without duplicate, then overload/direct FAILED must publish exactly one failed result and retain ownership. It fails in0.63s against old code, reproducing state12/results1. Build65 invalidates the previous report when leaving HOLD; repeated FAILED still emits no duplicates. Anchored goal/fixture preparation is kept in the existing force planning module; absent scene/TF rejects safely. No new motion cases or profile changes.
- Build65 all15 packages/full CTest across ten packages PASS (one configured numerical IK skip). The previously red no-actuator HOLD fault publication regression is green in0.64s; it verifies exactly one success, no healthy-HOLD duplicate, exactly one new overload failure, no repeated FAILED duplicates and retained ownership. Fresh read-only native preflight after64 remains healthy SDK2.3.3/controller3.9.3 FR5 label, Manual1/enabled1; no native actuator/mode/enable/reset calls. Mounted25 validation of current25 um/s common axial cap is running.

- Mounted25 revision64 still FAIL/SEATING MOVING (pitch122.306 urad/s, depth14.929 mm), transport healthy and controlled shutdown PASS. Lower axial speed did not change the contact impulse. Independent source poses move only2.52 urad over the selected two-wall-second window; exact16 ms finite pose derivative peaks98.32 urad/s (mounted24:97.97). Build66 evaluates bounded source-window pose velocity, retaining raw pose/drift/source/receipt guards. This deliberately changes measurement bandwidth, with up to16 ms sustained-motion response, not a zero-twist substitute or certification. Regression requires initial history, interpolation, child-frame velocities, antipodal handling, duplicate/gap/backward/NaN rejection, raw clock preservation, immediate2 um-step rejection and bounded-buffer failure. Mounted retract additionally requires observed parking owner0/state0 after successful retract. Physical acceptance is pending.
- Build66 all15 packages and complete CTest across ten packages PASS (one configured numerical IK skip). The pose-window regression passes at1/3/8 ms source cadences; raw pose and timestamps remain intact, duplicates do not renew, 2 um step triggers MOVING and sustained above-limit motion is detected by16 ms. Mounted26 is running the entire one-pose/HOLD/retract/parking-release trial. Fresh native after66 read-only transport PASS, Manual1/enabled1 and all error/safety fields0; actuation is still not admitted.
- Mounted26 Build66 passes physical parking through shoulder/contact, reaches VERIFY, but full trial FAIL/NO_PROGRESS at15.0135 mm; no HOLD/retract acceptance. Controlled stop, owned-node-first shutdown, zero LCP/unexpected contacts confirmed. Physical keyed fit116.17 um and penetration6.656 um satisfy geometry, but force verification does not. Raw-source replay diagnoses F/T aliasing: actual all-source last3-source-second mean load2.61 N, 1 Hz all-source filtered range1.868–3.266 N; 8-fold decimation at the same cutoff produces1.000–5.755 N and spuriously near-zero best error. The existing physical progress monitor then cannot improve that false historical minimum. Build67 conditions a bounded128-sample source batch before125 Hz consumption; tare still requires25 separate control observations, raw extrema/invalid packet safety remains independent, overflow invalidates evidence. Every sample reuses the latest bounded control-observation frame map (no per-sample FK/MoveIt/network or extra admittance integration). A no-actuator burst regression requires nonzero50%-duty mean when every last sample is0, then rejection of an invalid intermediate packet. Timing warnings for valid checks are throttled; invalid checks remain individually logged. Physical acceptance pending.
- Builds67/68 caught incomplete IK test-double methods; test69 initially used an invalid multi-axis AXIAL goal, and test70 incorrectly expected retained ownership before entering the socket. Test fixtures/assertions corrected without relaxing runtime barriers. Build71 all15 packages and complete CTest across ten packages PASS (configured numerical IK skip). New1.24-second no-actuator source-batch regression passes: the50%-duty sensor burst is not aliased to its zero last sample, and an intermediate NaN still yields INVALID_WRENCH + acknowledged stop. Existing HOLD failure publication, parking executor/freshness, geometry/rollback/motion and driver tests all pass. MuJoCo26 tilted/folded physical HOLD + overload is running.
- MuJoCo26 Build71 tilted_up/folded FAILS before HOLD at14.920 mm with raw FORCE_LIMIT (safety5), acknowledged stop/shutdown. No injected overload was sent. At source127.824 s the keyed physical pose is near target and only four shoulder contacts remain; at127.832 s the physical tool shifts~0.63 mm laterally and many nose/shoulder contacts appear, rawFx122.879 N. Recorded command J1/J6 stay0 and other increments remain microradians; no commanded lateral jump. Same live/planning model hashes, geometry and gains as MuJoCo25. First isolated cold-state replay does NOT reproduce the event, and its wall/source reference mapping is too slow, so it establishes no root cause. A discrete-integrator candidate produces worse604 N/1.09 mm penetration and is rejected; unchanged implicitfast retained. A250 us cold replay is smoother but does not prove recovery. Mounted27 runs current all-source F/T conditioning before further serial MuJoCo contact diagnosis.
- Mounted27 Build71 reaches VERIFY with healthy parking and acknowledged stop/shutdown, but FAIL/NO_PROGRESS at15.00694 mm; no retract attempted. All-source filtering removes the false low-error spikes and brings force near4 N, but instantaneous source-derived force rate remains roughly18.2 N/s median,54.9 N/s p95 and80.8 N/s maximum in the selected final2-source-second segment (estimated from receipt-aligned raw source timestamps). Current filtered force still has contact ripple near3.7–4.3 N, so no200 ms force/rate dwell. A200 ms finite trend replay lowers final slope to~0.07/0.51/2.25 N/s median/p95/max, but instantaneous force tolerance also needs margin. Build72/73 evaluates verification-only source-window slope over the existing completion dwell, retaining immediate ForceSafety rate/raw/overload/freshness protection, and reduces only axial drive deadband from0.75 to0.5 N inside unchanged1 N acceptance. Fixed64-observation history rejects insufficient history, backward/nonfinite/gapped sources and cannot shorten the window. VERIFY diagnostics expose both rates and actual geometry/tracking masks. Matched MuJoCo now publishes physics/FT/joint/clock each actual1 ms step via existing --period option (command executor remains125 Hz); previously its last-substep8 ms output was already decimated before conditioning. Native observer period remains8 ms pending actual SDK commissioning.

- Build75 all15 packages and full CTest across ten packages PASS (one configured numerical IK skip). Verification-only source-window slope regressions pass for ripple, true increasing force, off-grid source timing, duplicates, backward/gapped/nonfinite sources and insufficient bounded history. The seated HOLD regression now waits for the full200 ms measurement window plus200 ms verification dwell and stop acknowledgment; no acceptance thresholds were relaxed. Matrix06 current runtime passes24/24 station × initial configuration combinations with timed IK/FCL/jerk/braking checks; this remains read-only planning evidence, not24 physical insertions. Logs saved under validation75.
- MuJoCo27 front_mid/home with actual1 ms physics/FT publication FAILS before HOLD with raw FORCE_LIMIT(safety5) and controlled stop. Independent contact penetration peaks4.017 mm; no overload injection or successful HOLD was reported. Source-correct cold replay of MuJoCo26 still does not reproduce its original force burst. Smooth contact impedance across50 micrometres lowers the cold replay onset load while preserving original triangles/radius0; this is a candidate, not an accepted fix. New contact parameters are recorded in the shared simulator calibration YAML and runtime surface manifest. Mounted28 tests the Build75 verification changes using the pre-change source snapshot while MuJoCo contact calibration is being prepared separately.

- Mounted28 Build75 confirms one pose → preinsert → force seating → verified HOLD5-wall-seconds: INSERT success0 at15.016739 mm, physical contact/fit PASS, keyed error119.97 µm, station penetration10.17 µm, stationary FK76.89 µm, no unexpected world contact/LCP, controlled node-first shutdown. Full run remains FAIL: explicit withdrawal yields parking MOVING(safety14), then acknowledged stop; no parking release. Source-window16 ms pitch velocity crosses102.55 µrad/s and peaks173.32 µrad/s on load release, while translation stays below30 µm/s. New mounted insertion policy caps in canonical vehicle YAML reduce withdrawal velocity/acceleration/jerk and linear speed to0.5 mm/s; original100 µm/s and100 µrad/s base guards unchanged. Acceptance pending rerun.
- MuJoCo contact unit/physics9 tests PASS after preserving ROS PYTHONPATH. Contact load test now compensates gravity exactly as runtime, removing false seating force from gravity support. Smooth-onset static sweep at0/0.1/1/10/40 µm gives axial load~0/0.0000001/0.000045/0.191/11.550 N; rigid original triangles and radius0 remain unchanged. Positive contact assertion sampled at20 µm (>1 N), within unchanged50 µm physical gate. Calibration rejects malformed/nonfinite/unbounded profiles. This establishes bounded static onset and real seating load, not dynamic E2E acceptance. The initial test command accidentally replaced ROS PYTHONPATH and failed xacro discovery; rerun with preserved path passes.

- Build76 all15 packages + full CTest across ten packages PASS; existing launch regression confirms both mounted free-space and strict insertion policy values reach the actual node from canonical YAML. MuJoCo9 contact/model tests PASS. Fresh MuJoCo28 front_mid/home full HOLD/retract starts with smooth-onset calibration and1 ms actual sensor publication. Mounted29 withdrawal-cap acceptance will run serially after MuJoCo.

- MuJoCo28 Build76 front_mid/home smooth-onset FAIL/NO_PROGRESS before HOLD, with controlled stop and node-first shutdown. No first-contact overload: peak axial raw0.418 N, penetration6.457 µm, keyed error5.021 µm, post-stop physical drift0.791 µm. This is a large contact stability improvement in this measured case, not full acceptance. Failure follows the100 µm directional watchdog step in2 s, incompatible with25 µm/s seating correction and slowed contact approach; force improvement0.232 N also falls below the5%-of5 N objective step. Expose existing minimum_progress_m as a ROS/YAML parameter (default100 µm preserved), commission matched at10 µm;2 s timeout, force-improvement threshold and all HOLD/fault criteria unchanged. Existing monitor remains monotonic, so backtracking/phase cycling cannot renew it. New regression checks sustained25 µm/s progress and failure on stationary1 µm ripple. Build77 all15 packages PASS; full CTest running.

- Build77 all15 packages + complete CTest across ten packages PASS. Fresh MuJoCo29 front_mid/home **full PASS**: one pose → preinsert → force seating → verified HOLD5 wall seconds → explicit retract and owner release, node-first controlled shutdown. Depth15.000706 mm; physical keyed error59.509 µm, penetration11.788 µm, physical HOLD drift0.143 µm; preinsert retract depth error1.123 µm. Zero unexpected station contacts and no shutdown braking errors. This accepts this measured case under current smooth-onset +1 ms FT+calibrated watchdog profile, not all stations/native contact. MuJoCo28 failure retained. Read-only native125 Hz state observation and mounted29 full withdrawal-cap test follow serially.

- Native Docker read-only feedback soak after77 PASS:1250 SDK state reads over10 s at125 Hz,1240 new frame observations,0 read/unhealthy errors, maximum cached-frame age8.006 ms and read duration1.313 us. Mode Manual1/enabled1 unchanged,0 actuator commands. SDK3.9.3/controllerFR5 identity pinned to Docker192.168.58.2. This is feedback transport evidence only, not ServoJ soak or contact acceptance; cabinet Auto/explicit operator mode authorization still pending. Mounted29 current77 full cắm/HOLD/rút/parking-release test running on private domain228/port12109.

- Mounted29 Build77 again passes insertion/HOLD (depth15.024909 mm, keyed125.756 µm, penetration10.133 µm, no unexpected contact/LCP), but full withdrawal FAILS parking MOVING/safety14 after~4.4 source seconds of retract, controlled stop/node-first shutdown acknowledged. Uniform500 µm/s and low joint jerk do not adequately soften the loaded shoulder release. Build78 adds an optional bounded initial linear speed over100 µm, then quintic blending of the scalar Ruckig velocity ceiling over the next100 µm to ordinary withdrawal bounds. It remains one strict Cartesian trajectory and the same FSM/ownership; all joint/Cartesian derivative, FK/collision and braking validation still runs in the worker. Default disabled, matched arm-only25 µm/s, mounted2.5 µm/s from geometry YAML. Regressions check slow loaded start, faster remaining extraction, straightness/jerk, moving-seed controlled stop and invalid profile rejection. Physical acceptance pending.

- Mounted29 detailed timing:~29.14 wall seconds were worker planning/validation before any command changed; failure occurs~0.39 source seconds after trajectory activation, not4.4 seconds of actual movement. First command increments are nanoradians and preserve the stationary seed, so no initial seed jump was observed. The release transient occurs during the initial accelerated motion. Build79 corrects only the regression fixture velocity-scale calculation (dummy test limits0.20 m/s versus canonical0.80), retaining the same requested0.5 mm/s extraction cap. Build78/79 all15 packages PASS; full CTest79 running.

- Full CTest79 caught a real scalar timing failure for the new loaded-start cap; no physics launched. Solving the full45 mm at2.5 µm/s predicts18,000 s although only the first100 µm needs that speed. Build80 bounds the scalar target prediction horizon by the local capped speed, conservative braking time and twice the loaded distance, while retaining the original full endpoint and one dense trajectory. Build80 all15 packages + full CTest across ten packages PASS (one configured numerical IK skip). New tests pass slow initial cap, fast remaining extraction, straightness/derivative limits, admission of a compatible live seed without resetting it, controlled stop for an over-fast seed, and invalid configuration rejection. Mounted30 current80 physical loaded-start acceptance pending.

- Mounted30 passes the previously failing loaded release and has reached the socket mouth with contact FREE and healthy parking; full preinsert/owner-release still pending. Native CAD observer static regression PASS (no SDK/network/physics integration): observes exact measured q, gravity-compensated positive contact, preserves model time and rejects malformed/out-of-range input without mutation. Prepared observer replaces the second private torque servo with one quasi-static geometry/contact computation; real SDK JointState velocity/time/health and actuation remain untouched. Full latency and native Auto E2E pending.

- Mounted30 Build80 **full PASS**: one pose → preinsert → force HOLD5 wall seconds → one loaded-start/straight withdrawal trajectory → verified stationary preinsert → force/parking ownership released → acknowledged node-first shutdown. INSERT success0 at15.017339 mm, keyed123.468 µm, penetration10.132 µm, HOLD physical drift83.058 µm, retract depth error8.346 µm.0 unexpected world contacts/LCP/shutdown braking errors; parking_release_pass=true. Exact original CAD/chassis physics/mass and100 µm/s/100 µrad/s parking barriers preserved. Native observer standalone tests/benchmark and current MuJoCo tilted/folded acceptance follow serially.

- Native CAD observer standalone200-state20–40 µm static sweep PASS: exact measured q/model time preserved,0 actuator commands/no integration/physical_ft=false. Mean6.551 ms,p956.757,max8.024,all<50 ms; positive axial proxy1.499–11.550 N.25 combined model/observer/proof/evidence tests PASS. This proves kernel headroom on this host, not a full ROS/SDK actuation soak. Mounted30 16 ms source-window offline replay over entire withdrawal: peak translation23.67 µm/s, angular pitch81.56/roll53.67 µrad/s,below unchanged100 barriers. Retraction took140.55 source s/716.96 wall s. Preserve2.5 µm/s loaded region; optimize only ordinary extraction0.5→3 mm/s (existing arm-only calibrated cap),same low joint jerk. Build81 all15 packages PASS; full CTest81 running, then fresh tilted/folded MuJoCo and optimized mounted31 acceptance.

- Build81 all15 packages and full CTest10 packages PASS; loaded-start/seed/jerk regression includes ordinary3 mm/s extraction. Fresh tilted_up/folded MuJoCo30 full cycle running; optimized mounted31 remains pending.

- MuJoCo30 Build81 tilted_up/folded FAIL before contact at-6.888 mm: pending force validation expired, safety16, acknowledged controlled stop/node-first shutdown. No contact/penetration/overload. Recent worker checks~28–31 ms; control feedback shows repeated16 ms receipt intervals near fault, but1 ms joint-source stream near fault has no>8 ms gap. No conclusive scheduler/root-cause proof; complete original traces retained. Fresh same-runtime serial rerun will test reproducibility without compilation/planner load.

- MuJoCo31 Build81 tilted_up/folded full PASS: INSERT15.001225 mm, keyed66.231 µm, penetration11.879 µm, physical HOLD drift0.199 µm, retract depth error1.113 µm;0 unexpected contact, verified force release/node-first shutdown. MuJoCo30 validation deadline failure did not recur; this is not conclusive resolution of scheduling jitter. Harness strengthened to require5 measured joint-source seconds AND5 wall seconds before explicit retract/post-HOLD fault; frozen/slow/backward/nonfinite clock regressions6/6PASS. Existing5-wall-second runs remain historical with shorter physics duration.

- Mounted31 Build81 full PASS at normal3 mm/s: insertion/HOLD5.005 measured-source seconds/36.249 wall seconds, INSERT15.080410 mm, keyed122.777 µm, penetration9.740 µm, physical HOLD drift61.255 µm, retract depth error7.991 µm, parking released,0 unexpected world contacts/LCP/shutdown braking errors. Validated45 mm withdrawal duration57.840 source s versus134.728 at0.5 mm/s in run30; actual-duration replay pending.
- MuJoCo post-HOLD overload31 current81 PASS: actual insertion/HOLD5.001 source seconds/35.598 wall seconds, independent physical fit before injectionPASS(keyed59.491 µm,drift0.145 µm); new failure resultcode6/safety5 with confirmedcontrolledstop, no auto-retract. Penetration11.779 µm,0 unexpected contacts/shutdown brake errors, owned node stopped beforephysics. Matrix07 and fresh read-only native10 s soak now running; no physics/SDKactuation.

- Matrix07 current81 read-only PASS24/24;1 detour, maximum route wall1.902 s, maximum Jacobian condition8.539. Native fresh soak81 PASS1250 reads/10 s,1239 new frames,0 errors/unhealthy frames,maxcachedframeage8.003 ms,maxread1.232 us;modeManual1/enabled1unchanged,0actuatorcommands. Native actuation remains notaccepted. Mounted31 withdrawal replay:60.473 source s/386.057 wall s versusrun30 140.550/716.959; valid60077 source16 ms windows,32recorded windows lack usable history (notfilledwithzero). Observedmaxtranslation23.228 µm/s,angularpitch81.546/roll55.795 µrad/s. ActualruntimeparkingguardPASSindependently. ConditionedHOLDforce3.920–4.756 N;MuJoCo5-source-secondHOLDforce4.044–4.063 N. This is measured monitoredpositionHOLD, not evidence of exact5 N regulation throughoutHOLD.

- Handoff artifact/link auditPASS: current build/fullCTests,24/24matrix, mounted31/MuJoCo31/fullpost-HOLD-overload31 summaries and ownedshutdownallverified. README/pipeline/report now linkcurrentresults and distinguish physical-source duration, proxy/readonlyevidence and nativeManual blocker. No further runtime mutation aftervalidated81; no Mode/enable/reset/SDKactuation sent.

## Native Auto authorization and HOLD observation boundary, 2026-10-03

- User explicitly authorized selecting Auto if needed. Fresh Docker identity/SDK probes found Auto0/enabled1 already set, so no Mode, enable or alarm reset was sent.
- Native82 activated the sole v3.9.3 hardware plugin, performed one-pose insertion and reached a seated HOLD at15.029 mm. Zero SDK read/write/command errors; core125.005 Hz, no tick deadline misses or scheduler gaps. Static CAD proxy is not mechanical contact or a real FT sensor.
- Native82 remains FAIL: two delayed SUCCESS8 feedback samples after the independent success result caused the harness to refuse retract.512 subsequent HOLD9 samples and actual5.120 source seconds were recorded; this is diagnosis, not relabelled acceptance.
- Harness now awaits observed HOLD after success and a fresh measured joint/TCP boundary before starting both5 wall/source seconds. Any later non-HOLD feedback still fails. Focused clock/boundary/geometry tests7/7 PASS.
- Explicit recovery uses the same native driver/planner. First recovery sent no motion because its PlanningScene topic was wrong; second used the free-space pose topic, tripped the independent axis/clearance guard, and stopped cleanly after crossing the mouth. Both original FAIL artifacts are retained. Corrected recovery uses `/charging_port/raw_pose`, automatic insertion disabled only in this recovery overlay, reverse localaxis and0.5 mm/s strict INSERTION. Only the two intended mating pairs are allowed in ACM; protected rear and all other collision checks remain. No direct SDK recovery, teleport or container reset.

## Native full cycle and distant user pose, 2026-10-03

- Native83 full common-harness cycle PASS: INSERT15.0287 mm, HOLD5.008 measured-source /5.0066 wall seconds, explicit retract to−30.0005 mm, intended CAD contacts only, no SDK read/write/command errors or tick misses/gaps.125Hz loop observed124.992Hz. Original FAIL82 retained.
- Final harness boundary handles appended compliant states13–16 by transition semantics, not numeric ordering; monitoring begins at first observed HOLD, including events during the fresh-source barrier.27 identity/CAD observer/evidence tests PASS.
- Native84 inserted and recorded5.232 source seconds HOLD but subsequently FAILED before retract completion: hardware ServoJ returned14, then backend invalid feedback triggered COMM_FAILURE shutdown. Automatic reset/resume was not attempted. This is a failed run, irrespective of the earlier cached HOLD status.
- User subsequently reported moving the Docker robot farther away and explicitly requested motion to the correct socket. Fresh SDK measured q_deg=[104.221,−124.765,101.456,−126.511,−110.93,9.33098], Auto0/enabled1/no faults; canonical FR10 TCP=[0.164248,−0.398065,1.054516]m, mouth distance1.174121m, no CAD load. No old owned controller remained.
- Native85 uses fresh measured initial q and one canonical mouth pose. Planner validated293 IK waypoints/1884 timed samples/15.064s, max reported condition16.27, with full production collision/singularity/time-law validation. No explicit retract was requested for this run, matching the user's request to leave the robot at the charging point.


## Final distant-pose outcome and handoff, 2026-10-03

- Native85 PASS from the user's fresh distant posture: one pose, automatic validated preinsert, force insertion15.0287mm,5.032source/5.035wall seconds HOLD. No retract requested, robot left seated. Keyed72.34µm/penetration34.45µm/CAD drift2.683µm;0unexpected contacts/SDK errors; controlled/node-first shutdown PASS. Core124.996Hz; one scheduler gap12.459ms, max feedback age15.391ms;0execution deadline misses. Do not claim jitter eliminated.
- Exact readback uses full-precision joint JSON: SDKq_deg=[0,−86.89315032958984,122.23802185058594,−125.34465026855469,−90,0], FK=[−0.7199281117334161,−0.15899966471082666,0.30497028105485213]m; depth15.029719mm,77.790µm from nominal seated point. Matches the last native85 measured configuration, no additional motion commands. Ten-second read-only125Hz soak PASS:1250reads/1240fresh frames,0errors,Auto/enabled unchanged. Native cabinet remains enabled; ROS/Servo session closed.
- Latest complete CTest15package traversal:110tests=109PASS/1intentional numericalIK SKIP;27Python identity/CAD/boundary evidence checks PASS. No production C++ or runtime binary change after native83. Read-only probe serialization precision alone updated/compiled after native85, with separate final snapshot/probe hash in validation84.
- REPORT, README_LENH and pipeline updated with current Auto/native evidence and retained failures. No robot reset, cabinet enable/Mode change, direct SDK recovery, container reset or foreign Gazebo kill was sent.

## Native85 distant-motion quantitative analysis, 2026-10-03

- User asked duration/smoothness/velocity/jerk/singularity. Offline analysis only, no additional actuation. Received pose→preinsert22.6006s; logged motion start→stable goal15.1585s vs planned15.064s; planning/execution event delay7.5001s. Initial1.174m Euclidean mouth distance differs from1.38677m commanded FK TCP path length.
-1895 accepted qva reference samples, no missing ticks in approach. TCP commanded finite-difference peak99.959mm/s, acceleration≈0.4134m/s², jerk≈1.8195m/s³; respective caps100mm/s/0.88388m/s²/1.875m/s³. SDK qdot/FK peak100.238mm/s. Raw SDK position/source finite differences peak112.733mm/s, a different noisy sampled estimator, preserved in JSON rather than discarded. Physical jerk is not measured.
- Command derivative estimates use nominal8ms sequence increments, whereas executor uses measured period; values are approximate sampled peaks. J1 qva velocity0.124638rad/s, acceleration0.537222rad/s², sampled jerk≈2.337929rad/s³ vs limits0.375/1.75/5. SDK fastest jointJ1 peak0.124921rad/s. Jacobian condition logged16.27<30; wristQ5 minimum90°>8°, elbowQ3 minimum101.46°>15°. No singularity barrier or detour was needed in this run.
- Reproducible metrics/curves/figure added under native_85_far_pose, scripts in validation84; REPORT includes methods, limits and the previously recorded full-run scheduler gap.
