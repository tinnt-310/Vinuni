# Workspace module-granularity audit — in progress (2026-09-27)

Status: **not accepted** against master task §21. This is the first evidence-backed slice; the mandatory per-file, eight-axis matrix for every in-scope file is not yet complete. Do not use this document to claim architecture PASS.

## Scope and inventory

`rg --files src/fr10_arm_control src/vehicle_description src/vehicle_bringup src/swerve_vehicle_control src/hardware_interface tools` lists 358 files: 342 excluding `.orig`/`.rej` patch artifacts. Of those, 263 have source, configuration, model or interface extensions (`.cpp/.hpp/.h/.py/.xacro/.yaml/.xml/.world/.sdf/.urdf/.srv/.msg`, `CMakeLists.txt`, `package.xml`); the rest include meshes, maps, libraries, documentation and visualization assets. Binary/vendor assets require ownership and reference checks, not textual function-level cohesion analysis. Patch artifacts are tracked separately as delete candidates, never treated as production source.

## Observed layer and runtime boundaries

```text
fr10_motion (C++ core; Ruckig/Eigen, no ROS)
  <- fr10_backends (rclcpp RosControlBackend)
  <- fr10_pose_controller_node (ROS inputs, MoveIt planning, 125 Hz control)
  -> ros2_control command topic -> FAIRINO hardware plugin/SDK

vehicle_bringup/simulation.launch.py
  -> Xacro + GazeboSystem -> joint_trajectory_controller / swerve_drive
  -> Gazebo ODE + wheel_contact_surface ModelPlugin

tools/ diagnostic clients/observers -> ROS/Gazebo public interfaces only
plc_bridge -> separate Python PLC runtime, not part of FR10 motion core
```

The simulation launch does not run `motion_adapter.cpp`; the ODE vehicle failure and FR10 orchestration coupling are distinct. `RobotBackend` is a domain-facing actuator port; `RosControlBackend` is its ROS implementation. The physical FAIRINO plugin is the sole SDK actuation route in the reviewed FR10 path. `fr10_motion` is a single shared-library compilation boundary; `fr10_pose_controller_node` currently compiles `motion_adapter.cpp`, MoveIt provider and parameter parser into one executable. `swerve_drive` executable and `swerve_controller_plugin` shared library are both built, but the Gazebo launch selects `swerve_drive`.

## Eight-axis review: critical files (first slice)

The columns correspond to §21.1–21.8. “Pending” means evidence has not yet been collected; it is not an OK verdict.

| File | Responsibility / cohesion | Coupling / direction | State ownership | Unit testability | Compilation boundary | Runtime owner | Verdict |
|---|---|---|---|---|---|---|---|
| `fr10_pose_controller/src/motion_adapter.cpp` | ROS lifecycle, input decoding, insertion policy, planning worker, executor activation, safety monitor, diagnostics and signal handling: multiple reasons to change. | High fan-out into pure core, MoveIt, ROS and backend; node invokes domain decisions directly. | Owns target/sequence/insertion/scene/stop snapshots alongside `MotionExecutor` motion state; invalidation paths require detailed review. | Requires ROS/MoveIt for most methods; extracted tracking and trace validation are independently tested. | Rebuilds large node TU for any policy/diagnostic change. | `fr10_pose_controller_node`, not launched in Gazebo scenario. | **UNDER_FRAGMENTED**; extract cohesive pure policy and planning service, not file-size wrappers. |
| `fr10_pose_controller/src/trajectory_trace_validator.cpp` + header | Validates every timed trajectory sample through IK/collision provider and FK trace; one reason to change. | Depends on domain IK/trajectory/math only; no ROS/Gazebo/SDK. | Stateless. | ROS-free `test_trajectory_trace_validator` passes. | Part of `fr10_motion` shared library; appropriate for reusable domain rule. | Called by planning worker, not 125 Hz tick. | **OK** for this responsibility; inspect per-file include fan-out in full matrix. |
| `fr10_pose_controller/src/linear_insert_planner.cpp` + header | Builds a straight INSERTION/RETRACT segment and validates its measured axis; one geometric-policy reason to change. | Depends on CartesianPath/math/core only, no ROS/MoveIt/Gazebo. | Stateless. | ROS-free `insertion_planners` covers forward, retraction, lateral, reverse and nonfinite axis inputs. | Part of `fr10_motion`; correct reusable domain boundary. | Called from pose callback and, currently, the staging transition inside `controlTick`. | **PARTIAL**: cohesive domain owner, but generating a dense segment from the 125 Hz tick is an inherited scheduling violation to move off tick. |
| `fr10_pose_controller/include/fr10_pose_controller/cartesian_tracking.hpp` | Computes Cartesian tracking errors shared by monitor and diagnostics. | Domain math/core only. | Stateless. | ROS-free `test_cartesian_tracking` passes. | Header-only; may rebuild consumers, but small pure calculation. | Invoked in node monitoring/diagnostics. | **OK** provisionally. |
| `vehicle_bringup/src/wheel_contact_surface.cpp` | Sets four vehicle wheel ODE collision slip values at model load. | Gazebo/ODE plugin API only; no FR10 domain dependency. | Owns contact-surface configuration only. | Requires Gazebo integration; runtime surface values were read directly. | Separate Gazebo shared library. | Gazebo model plugin for vehicle, not standalone FR10. | **OK** provisionally; crash/repeat gate remains. |
| `vehicle_bringup/launch/simulation.launch.py` | Builds Xacro, runtime controller YAML, world and multi-model launch graph; multiple reasons to change. | ROS launch, Xacro, model packages, controller configurations; high fan-out. | Owns startup ordering, not motion/task state. | Integration-test only; startup race observed during simultaneous station spawn. | Python launch script; no C++ rebuild. | ROS launch process. | **UNDER_FRAGMENTED**; distinguish model generation from process orchestration without multiplying forwarding wrappers. |
| `swerve_vehicle_control/src/swerve_drive.cpp` | Runs the selected vehicle command/odometry process. | ROS topics/joint states; production Gazebo launch selects it. | Owns selected swerve command state. | Needs ROS integration; pure kinematic extraction may help if nontrivial duplication is confirmed. | Standalone executable. | `swerve_drive` process. | **PENDING** method-level review. |
| `swerve_vehicle_control/src/swerve_controller_plugin.cpp` + header | Alternate ros2_control swerve controller implementation. | Controller API/ROS; not selected in current Gazebo launch. | Would own a competing command path if activated. | Needs controller-manager integration. | Separate shared plugin. | Inactive in tested launch. | **DELETE_CANDIDATE / duplication review**; do not delete without user approval. |
| `hardware_interface/fr10_driver/fairino_hardware_v3_9_3/src/fairino_hardware_interface.cpp` | ros2_control-to-FAIRINO SDK bridge. | Hardware adapter depends on SDK and ros2_control, not on Gazebo. | Sole physical ServoJ command owner. | Requires SDK/SimMachine integration; ABI test verifies deployed library resolution. | Isolated `fairino_hardware` plugin linked to versioned SDK. | `controller_manager` hardware plugin. | **PENDING** method-level review; preserve sole-actuator rule. |

## Next audit work

1. Complete one row per source/config/model/interface file, explicitly record eight axes, include/import fan-in/out and verdict; inventory assets/references separately.
2. Inspect joint/target/task state ownership and whether any two small files always change together (over-fragmentation).
3. List each `motion_adapter.cpp` method with dependency, blocking behavior, runtime owner and test coverage before further architecture edits.
4. Add a consolidated delete-candidate table; no production file is deleted without approval.

The current architectural acceptance is **FAIL/PARTIAL** until these steps and resulting refactors pass build, pure tests, SimMachine safety tests where available, and Gazebo regression.

## Inactive FAIRINO command-server path (source inventory finding)

The `fairino_hardware_v3_9_3/CMakeLists.txt` compiles only `src/fairino_hardware_interface.cpp` into `fairino_hardware`; it contains only commented-out references to `ros2_cmd_server`. `src/command_server_node.cpp` includes `command_server.hpp`, but no built target includes that node or the 12,129-line `command_server.cpp`. This is source present in the workspace, not a second active SDK actuation path. The three files remain a **DELETE_CANDIDATE group requiring user approval**, not files silently removed for line-count reasons.

| File | Responsibility | Cohesion | Coupling (observed) | Dependency direction | State ownership | Testability | Compilation boundary | Runtime ownership | Verdict / recommendation |
|---|---|---|---|---|---|---|---|---|---|---|
| `fairino_hardware_v3_9_3/src/command_server.cpp` | Implements the retained command-server protocol. | Method-level cohesion pending; size alone is not a verdict. | Includes `command_server.hpp`, SDK/ROS and socket APIs; only the inactive node references the header outside this pair. | Hardware edge, not domain core. | Legacy server globals and reconnection state; do not share with active plugin without review. | Needs SDK/ROS fixture. | Excluded from all active CMake targets. | None in current launch/build graph. | **DELETE_CANDIDATE** after checking deployments and user approval. |
| `fairino_hardware_v3_9_3/include/fairino_hardware/command_server.hpp` | Declares the retained command-server API. | Interface corresponds to its implementation; method-level review pending. | Included by `command_server.cpp` and inactive `command_server_node.cpp`. | Hardware/ROS edge. | Declares legacy server state. | Compile/interface tests would need SDK headers. | Header not consumed by active target. | None in current graph. | **DELETE_CANDIDATE** with implementation; do not delete independently. |
| `fairino_hardware_v3_9_3/src/command_server_node.cpp` | Starts the retained command-server and CNDE receiver nodes. | One entry-point responsibility. | Includes command-server and CNDE headers. | ROS/hardware edge. | Creates both legacy nodes. | Integration-only. | No executable target in current CMake. | None in current graph. | **DELETE_CANDIDATE** with legacy route after deployment check and approval. |
