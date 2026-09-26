# FR10 architecture and ODE audit (baseline, before changes)

Date: 2026-09-26. This is the pre-edit inventory and proposed design; results after changes belong in the final report. The workspace root and `src/` have no `.git`, so a line-by-line historical copy/paste count cannot be established from Git. The `AGENTS.md` reference to `src/fr10_arm_control/doc/MASTER_SYSTEM_SPECIFICATION.md` is broken; that file is absent.

## Actual execution paths

```text
FAIRINO/SimMachine runtime:
JointState /joint_command ---------> input validation -> managed planning worker
PoseStamped /charging_port/raw_pose -> pre-insert policy -> route/IK/time law
    -> immutable joint trajectory -> MotionExecutor (125 Hz)
    -> RobotBackend -> RosControlBackend -> ros2_control command topic
    -> fairino_hardware_v3_9_3 (sole ServoJ owner, sim or physical cabinet)

Gazebo Classic vehicle simulation (independent controller graph):
vehicle_bringup/simulation.launch.py -> Xacro + GazeboSystem
  -> joint_trajectory_controller -> FR10 position joints
  -> swerve_drive -> steering/drive group controllers -> vehicle joints
  -> Gazebo ODE physics
```

The Gazebo launch does **not** start `fr10_pose_controller_node` or `motion_adapter.cpp`. Therefore its observed LCP errors cannot directly be attributed to a command emitted by that node. The two paths share the robot geometry and some ROS controller concepts, but have different command generators.

## Responsibility and dependencies

| File or cohesive file pair | Current responsibility | Layer / direct dependencies |
|---|---|---|
| `core_types.hpp`, `math_utils.hpp` | Motion data contracts and shared finite/angle/quaternion math | Domain; C++ standard library |
| `ik_provider.hpp`, `fr10_analytical_ik.*`, `continuous_ik.*` | IK abstraction, FR10 closed-form branches, continuous branch selection | Domain; core types/math |
| `cartesian_path.*`, `cartesian_route_planner.*` | Dense Cartesian samples and collision-aware route geometry | Domain; IK abstraction/math |
| `cartesian_trajectory.*`, `cartesian_trajectory_detail.inc` | C2 path and jerk-limited time parameterization | Domain; core/IK, Eigen |
| `pre_insert_planner.*`, `linear_insert_planner.*` | Staging point and strict insertion geometry | Domain; core/math |
| `joint_otg.*`, `motion_controller.*` | Joint Ruckig generation and single joint-only executor | Domain; Ruckig/core |
| `realtime_buffer.hpp`, `diagnostic_display.hpp` | Bounded handoff and diagnostic pose formatting | Infrastructure and pure display math |
| `moveit_ik_provider.*`, `collision_validated_ik_provider.hpp` | MoveIt FK, kinematic/collision verification, analytical + scene composition | Planning adapter; MoveIt/domain interfaces |
| `node_parameters.*`, `controller.yaml` | ROS parameter parsing and motion limits/policies | Configuration adapter; ROS/domain |
| `robot_backend.hpp`, `ros_control_backend.*` | Actuator interface and ROS command/state transport | Port and transport adapter; ROS/domain |
| `motion_adapter.cpp` (2,741 lines) | ROS lifecycle, all input parsing, policy staging, worker scheduling, trajectory planning/validation, activation, path monitoring, diagnostics, transport | Mixed orchestration/domain/ROS/MoveIt; 1 class with ~30 methods and many mutexes/buffers |
| `fr10_pose_controller_node.cpp` | Calls node entry point | Executable entry |
| `fairino_hardware_interface.*` | Hardware plugin and FAIRINO SDK actuation | Driver; ros2_control/vendor SDK |
| `vehicle_bringup/simulation.launch.py`, `simulation.xacro` | Compose standalone/mounted Gazebo models/controllers | Simulation infrastructure |
| `vehicle_chassis.xacro`, `swerve_module.xacro`, `thanxe_new.gazebo`, `vinfast_map.world` | Physical geometry/contact parameters/world | Simulation physics |
| `swerve_drive.cpp`, `swerve_controller_plugin.cpp` | Two swerve implementations; launch currently selects `swerve_drive.cpp` | Vehicle control; ROS/controllers |

No include-level cycle was found in the controller core: domain headers do not include ROS or Gazebo, MoveIt adapters depend on domain interfaces, and the backend implements `RobotBackend`. The large node class defeats that otherwise clean dependency direction: domain decisions are made inside ROS callbacks and worker methods. `robot_backend.hpp` is already the shared sim/physical transport port; the FAIRINO hardware plugin, rather than two competing SDK clients, selects the physical or SimMachine destination. Gazebo has its own GazeboSystem implementation of the ros2_control contract.

## Algorithms, validation and duplicates

| Algorithm | Source of truth | Input guard / finding |
|---|---|---|
| FK/8-branch analytical IK and Jacobian singularity barriers | `fr10_analytical_ik.cpp`; cross-check against `MoveItIKProvider` | Joint finite/limits and reach checks in provider; independent MoveIt verification at startup |
| IK continuity and path candidates | `continuous_ik.cpp`, `cartesian_route_planner.cpp` | Config/waypoint and validation errors propagated; PlanningScene checked on worker |
| Cartesian interpolation and insertion | `cartesian_path.cpp`, `pre_insert_planner.cpp`, `linear_insert_planner.cpp` | Finite pose/quaternion and geometry checks; policy sequencing still lives in node |
| Speed/acceleration/jerk limits | `JointOtg`, `PathTimeParameterizer`, `MotionExecutor`; values from YAML | Generated samples validated; no duplicate hard-coded deployment limits found in this path |
| ROS command/state conversion | `RosControlBackend`, `motion_adapter.cpp` callbacks | Backend rejects nonfinite command position/velocity/acceleration; pose/joint callbacks reject nonfinite targets |
| Cartesian error and along-track projection | `motion_adapter.cpp` path monitor and diagnostics | Duplicated arithmetic; risk of different error definitions between stop guard and telemetry |
| Pose timestamp/finite/quaternion checks | `poseTargetCallback` and `queuePoseTarget` in same file | Duplicated validation; stage target is reserialized into a ROS message and parsed again |
| Pose distance and orientation tolerance | Three places in `motion_adapter.cpp` | Duplicated arithmetic for duplicate target, PRE_INSERT and measured boundary |
| Swerve wheel kinematics | `swerve_drive.cpp` and unused `swerve_controller_plugin.cpp` | Similar control equations in two vehicle implementations; plugin is not launched here |

The current `motion_adapter.cpp` is a God file by responsibility, even though individual domain algorithms were split out. The historical claim that code was copied from the old `fr10_pose` is **unverified** without old source/history. File size is evidence of current coupling, not proof of provenance.

## Proposed target boundaries

```text
ROS node: subscriptions/timers/lifecycle, snapshots and publishing
  -> command decoding/validation (one pure conversion boundary)
  -> pose sequence coordinator (PRE_INSERT, INSERTION, RETRACT state)
  -> planning service (scene snapshot, route, IK, time law, FK/collision check)
  -> MotionExecutor (only 125 Hz state/trajectory consumer)
  -> RobotBackend -> ros2_control -> GazeboSystem or FAIRINO plugin
```

Extract independent math and policy first, with pure C++ tests; move planning/telemetry out of the node without creating `.inc` fragments that merely relocate lines. Keep public topics, control rates, safety limits and SDK calls. Do not add ServoCart or a second FAIRINO driver. Build both controller and hardware plugin, then CTest and separate Gazebo regressions.

## ODE baseline evidence and next isolation

`DEBUG_ODE_REPORT.md` records 552 LCP errors in the old arm baseline and 0 in each of three runs after static Dumpster and base collision clearance changes. In the mounted case it records 331 LCP errors, 263 with arm collisions removed, and 3,707 when wheel collisions were removed. `vehicle_final_1/gazebo.log` shows errors immediately after steering controller activation; the Gazebo launch has no pose node. This points to the wheel/chassis/contact system, but does not yet isolate the exact contact or gain. The world uses ODE world solver, 1 ms step, CFM 1e-5, ERP 0.2; wheel contacts use kp 1e6 on wheels 1-3 and 1e7 on wheel 4. A fresh vehicle-only run and controlled one-variable experiments are required before changing physical parameters. Existing `run_ode_regression.py` always starts the arm; it does not cover vehicle-only or vehicle drive acceptance.
