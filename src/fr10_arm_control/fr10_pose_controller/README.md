# fr10_pose_controller

ROS 2 Humble motion controller for the FAIRINO FR10. The public API exposes
exactly two control modes, `ControlMode::JOINT` and `ControlMode::POSE`. They
share one 125 Hz joint-only executor and reach the robot through
`ros2_control`. MoveIt Servo and Cartesian Servo are intentionally not used.

The current matched force insertion contract and commissioning evidence are
documented in [the insertion specification](../../../docs/fr10_insertion_pipeline.md).
With `force.automatic_pose: true`, `/charging_port/raw_pose` is the socket-mouth
pose and autonomously runs preinsert, six-axis compliance, seating verification
and stationary HOLD. The position-only legacy policy below remains available
when force is disabled. Free POSE uses `/fr10_motion/pose_target`.

While a force session owns motion, tracking logs display `FORCE` and its phase,
for example `[TRACKING | FORCE | APPROACH_COMPLIANT | 125.0 Hz]` or
`[HOLDING | FORCE | FT_ZERO | 125.0 Hz]`. Internal pose/joint handovers do not
change this task label. Force HOLD, stopping and retained faults keep the label
until ownership is released; enabling force without an active session leaves
ordinary POSE/JOINT logs unchanged.

The sole supported runtime backend is
`fairino_hardware_v3_9_3/FairinoHardwareInterface`. It uses its colocated SDK
2.3.3 for both Docker SimMachine and a physical controller;
`fr10_pose_controller` does not link or install another FAIRINO SDK.

## Implemented architecture

```text
ControlMode::JOINT                 ControlMode::POSE
/joint_command                     /charging_port/raw_pose (one final Pose)
  -> Ruckig Joint OTG                -> PreInsertPlanner
                                      -> policy FREE_SPACE: CURRENT->PRE_INSERT
                                      -> HOLDING with qdot=qddot=0
                                      -> policy INSERTION: PRE_INSERT->TARGET
                 |                                  |
                 +----------> MotionExecutor <------+ 
                              -> ros2_control -> ServoJ 125 Hz
```

For the position-only legacy policy above, `FREE_SPACE` and `INSERTION` are
`PoseExecutionPolicy` values, not public modes. Its `PRE_INSERT` is a configured
250 mm geometric waypoint. The matched force profile instead uses the station
preinsert clearance (currently30 mm), qualified no-contact tare, six-axis
admittance/search and a measured accepted corridor for explicit withdrawal.
The force session still issues only validated targets to the same MotionExecutor.
Every position-only
single accepted Pose autonomously queues `CURRENT -> PRE_INSERT`, waits for a
settled HOLDING boundary, then queues the strict linear insertion. Retraction
runs the retained INSERTION line in reverse to PRE_INSERT.

`libfr10_motion.so` contains ROS-independent motion types, math, Cartesian path
generation, the fixed-size eight-branch FR10 analytical IK provider, insertion
geometry planners, continuous IK orchestration, Joint OTG, and MotionExecutor.
MoveIt and ROS adapters are built into the node/backend layer.

The Pose worker selects route geometry from the internal policy. Free-space
staging is solved to a joint goal and materialized as a dense joint-space
route, so TCP motion is not constrained to a Cartesian chord. Insertion and
retraction build a strict Cartesian line with continuous IK; all positional,
orientation, and joint-space detours are disabled for that segment. A failed
insertion IK/singularity check rejects the segment and requires PRE_INSERT
repositioning. MoveIt PlanningScene monitors dynamic scene, collision-object,
attached-object, and world-geometry updates. Every candidate waypoint and
every timed sample must pass self/world collision validation on the managed
planning worker before activation. A plan is checked against one immutable
PlanningScene snapshot and carries its scene revision. A stale result is
replanned; a collision-relevant scene update during active motion requests a
controlled stop.

Free-space planning ranks complete insertion-capable IK branches, tries dense
direct joint interpolation, then uses bounded RRTConnect for intermediate joint
waypoints. Dense FK/collision validation is authoritative; a rejected coarse
solution triggers finer sampling and another search within the same budget.
The core still provides its deterministic cylindrical-route API for legacy
callers; every route retains the configured singularity and collision barriers.

The analytical geometry is cross-checked against independent MoveIt FK at
startup. `PathTimeParameterizer` applies one
jerk-limited scalar progress law to a C2 spline through the complete continuous
IK path and samples it at 8 ms. The operational triad uses velocity factor
`k`, acceleration factor `sqrt(k)`, and jerk factor `k`, with a minimum
200 ms ramp boundary. Every sample is checked against joint and
Cartesian limits and validated through MoveIt FK before the immutable buffer is
prepared and handed to `MotionExecutor`; route planning, IK search, structural
scans, spline construction, and MoveIt FK stay on the managed worker. The
125 Hz tick uses closed-form FK and a constant-time insertion-axis check at the
measured PRE_INSERT boundary, without materializing a path. Mid-motion planning seeds
the new trajectory with the live generated velocity and acceleration rather
than forcing them to zero. Constraint accounting is per joint; one joint's
slope is no longer compared against another joint's limit. The arbitrary
normalized `s <= 1/s` ceiling was removed, so duration is derived from the
configured physical limits and active mode scaling. Each timing plan emits one
`[TIME_PARAM_DIAGNOSTIC]` block with distance, rotation, duration,
joint/Cartesian utilization ratios, the initial scalar velocity cap, and the
limiting constraint.

The node runs a six-thread `MultiThreadedExecutor` with mutually-exclusive
control, target, feedback, stop, path-monitor, and diagnostics callback
groups. Kinematics has a separate managed worker. Cross-thread feedback,
planned results, commands, and diagnostic snapshots use bounded two-slot
buffers. The hardware broadcaster is the sole `/joint_states` publisher; the
adapter only subscribes to it. This separation does not by itself provide
real-time OS priority or CPU affinity.

At runtime, both JOINT and FREE_SPACE are supervised by the independent
joint-reference lead guard. A fresh sample beyond 1 degree starts one
jerk-limited controlled stop immediately; it never freezes and releases the
trajectory setpoint. Cartesian pose differences remain diagnostic in
FREE_SPACE because Mode 1 is deliberately free-form joint-space motion.
INSERTION alone enables the 125 Hz Cartesian monitor, which keeps perpendicular
deviation, along-track lag, and orientation error separate and requires 24 ms
persistence before requesting a controlled stop. This separation prevents a
curved FREE_SPACE route from being silently constrained back to a Cartesian
chord while retaining the strict 2 mm/1 degree insertion invariant.

There is no separate one-shot/stream mode. One valid Pose runs to completion
and HOLD. A changing Pose stream retargets the same trajectory; after input
stops, the last accepted Pose still runs to completion and HOLD. The 0.40 s
message-age parameter rejects stale messages when received and is not a stream
silence watchdog.

## Runtime interfaces

- `/joint_command`: `sensor_msgs/msg/JointState`; names must contain `j1` to
  `j6` exactly once. Values are degrees by default; set
  `controller.joint_input_unit: radians` for REP-103 input.
- `/charging_port/raw_pose`: reliable `geometry_msgs/msg/PoseStamped` socket
  input. Frame must equal `fr10_base_link`, position is metres, orientation is
  XYZW. With force enabled/automatic it denotes the socket mouth and runs the
  matched force session; otherwise it runs position-only staging/insertion.
- `/fr10_motion/pose_target`: reliable `geometry_msgs/msg/PoseStamped` free
  pose input through the same worker and executor; rejected while the force
  session owns motion.
- `/fr10_motion/retract`: reliable `std_msgs/msg/Empty`; executes the retained
  insertion line in reverse to PRE_INSERT without switching to free-space.
- `/fr10_motion/controlled_stop`: `std_msgs/msg/Empty`; explicitly requests a
  bounded Ruckig deceleration to zero reference velocity and then HOLDING. It
  is a software controlled stop, never a hardware E-stop.
- `/joint_states`: synchronized actual robot state from `ros2_control`.
- `/fairino10_controller/commands`: joint-only 125 Hz position-setpoint sink.
- `/controller_status`: 10 Hz state, owner, errors, loop rate/jitter, IK
  latency, and stop reason.
- `/fr10/diagnostics/tracking`: transient-local copy of the tracking status,
  including `loop_samples` and `mean_cycle_period_ms` for automated soak
  acceptance.
- `/fr10_motion/tcp_pose`: FK feedback published at 10 Hz.

Pose trajectories carry their generation, scene revision, planning timestamp,
and seed joint state. If asynchronous planning finishes after the live state
has moved beyond the activation-continuity bound, the result is classified
STALE and replanned from current feedback without entering FAULT. Pose
playback advances by one nominal 8 ms sample per successful control tick. A
late callback is recorded as jitter and never skips ahead across several
position references.

If a moving retarget has negative or incompatible velocity projection on the
new path tangent, or if scalar time scaling remains infeasible with a nonzero
moving boundary, timing returns `REQUIRES_CONTROLLED_STOP`. The node retains
the target, performs jerk-limited deceleration to zero, enters HOLDING, and
replans from that boundary. Timing-stop retries are capped at one per target;
a stationary failure is rejected without relaxing any limit. Stale activation
replans have their own bounded recovery path.

Cartesian monitoring keeps perpendicular error, along-track lag, and
quaternion orientation error separate. It runs in a non-control callback and
requires the configured persistence interval (24 ms by default) before a soft
path fault latches a generation-qualified request. Only the control runner
changes executor state. INSERTION is queued only after fresh measured FK also
confirms the PRE_INSERT position and orientation boundary.

The console evaluates one unified diagnostic block per second by default
(`controller.diagnostics.status_log_rate_hz`) and prints it only when the
displayed state, mode, TCP, mechanical flange, joint values, or errors change.
An unchanged IDLE/HOLDING snapshot is therefore printed once; loop-rate jitter
alone does not repeat it. ROS status/TCP publishers and safety monitoring keep
their normal rates. Cartesian display angles follow the FAIRINO fixed-axis
X/Y/Z convention in degrees; pose error remains a quaternion distance
internally, and joint error uses wrapped radians internally.

The arm geometry and D435/CCS2 wrist are owned by
`vehicle_description/urdf/vehicle.urdf.xacro` for standalone and mounted
runs. The arm root is `fr10_base_link`, the commanded TCP is
`charging_tcp`, and the diagnostics mounting-face frame is `plate_frame`.
Tool 0 is the centre of the mechanical flange face (`plate_frame`); Tool 1
is the centre of the front mating surface (`charging_tcp`), not the tool CoG.
The sole calibration is `vehicle_description/config/fr10_tool.yaml`:
`plate_frame -> charging_tcp` translation `[0.055140, -0.046496, 0.215297] m`
and fixed-axis XYZ rotation `[0, 0, 136.840] degrees`. Thus both +Z axes agree.
All geometry representations use this same transform; mesh-local front-face
offsets only position the mesh behind that TCP. Launch files also derive the
wrench rotation and moment arm from the expanded description. The
upstream `wrist3_link -> plate_frame` joint adds
`[0.000, 0.000, 0.106] m` and a 180-degree Z rotation. Runtime FK and
diagnostics resolve these transforms from `robot_description`; they do
not keep a second hardcoded tool offset. The model's bracket and connector
collision/inertia approximations need physical verification before hardware
TCP accuracy can be accepted.
Manual analytical-IK tool transforms and disabling `use_model_tool_transform`
are rejected at startup, so the runtime cannot select a second tool calibration.

Parameters and engineering limits are in
[config/controller.yaml](config/controller.yaml). They are not certified robot
safety limits.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select fairino_hardware_v3_9_3 fr10_pose_controller
ctest --test-dir build/fr10_pose_controller --output-on-failure

# Optional local GenericSystem smoke diagnostic (not runtime acceptance)
PYTHONPATH=src/fr10_arm_control/fr10_pose_controller/test/simmachine:${PYTHONPATH} \
/usr/bin/python3 src/fr10_arm_control/fr10_pose_controller/test/simmachine/test_fake_runtime.py

# Optional local deterministic GenericSystem diagnostic
PYTHONPATH=src/fr10_arm_control/fr10_pose_controller/test/simmachine:${PYTHONPATH} \
/usr/bin/python3 \
  src/fr10_arm_control/fr10_pose_controller/test/simmachine/test_extended_fake_runtime.py \
  --results-dir test_results/extended_fake

# Offline kinematic/control feasibility basin (CSV, JSON, SVG)
PYTHONPATH=src/fr10_arm_control/fr10_pose_controller/test/simmachine:${PYTHONPATH} \
/usr/bin/python3 \
  src/fr10_arm_control/fr10_pose_controller/test/simmachine/test_extended_capture_basin.py \
  --output-dir test_results/extended_basin
```

The extended CTest matrix leaves the unavailable MoveIt numerical-IK benchmark
at skip code 77. PlanningScene collision validation has a deterministic dynamic
object fixture. Shared force math/FSM tests and simulation spring-contact tests
are described below; they do not certify physical connector mating.

## Shared force-position insertion (simulation-only, default OFF)

`/fr10_force/insert` uses `fr10_motion_interfaces/action/Insert`. It is an action,
not a third public `ControlMode`; JOINT/POSE commands are BUSY while it owns motion.
Cancel/controlled stop have priority. Command ownership remains in MotionRuntime.

```text
WrenchStamped → validation/raw limits → sign/tare/gravity → moment-arm transform
             → insertion axes → low-pass → contact/safety
nominal TCP → shared ForcePositionController → corrected TCP → ContinuousIKStepper
            → FORCE_ONLINE MotionExecutor/Ruckig → sole joint-position backend
```

`fr10_force_control` and the insertion FSM/session contain no ROS/MoveIt/SDK API.
ROS adapters consume `/ft_sensor/wrench`, optionally `/ft_sensor/source_status`,
and expose `/ft_sensor/tare`, `/ft_sensor/status` and `/fr10_force/*` diagnostics.
Filtered wrench, error and correction use the session-specific
`fr10_force/insertion_axes_at_tcp_<generation>` frame: fixed insertion orientation,
origin at the measured TCP. Its TF is published with the accepted sensor stamp.
`correction` uses TwistStamped as displacement telemetry (metres / rotation-vector
radians), not a commanded twist. Nominal/corrected poses use `fr10_base_link`.
`joint_reference` reports acknowledged q/qdot/qddot; it is NOT an actuator channel.

The goal defines one primary insertion axis; the matched profile selects all six
admittance axes. A worker validates an axial
collision corridor and sampled braking states against a pinned scene, allowing
only the configured connector/socket proxy contact pair. This is sampled
validation, not a certified dynamic-envelope proof. Completion requires measured
depth tolerance, force error/stability dwell and no fault. Faults never auto-retract.
HOLD completion requests acknowledged controlled deceleration, freezes the
admittance state, keeps wrench safety active, and only returns success at a
stationary executor boundary. The matched commissioning harness then verifies five source seconds of stable
measured HOLD and independent keyed geometry before explicit measured-axis retract. Executor
diagnostics may say `STOPPED_BEFORE_GOAL` because this boundary uses the shared
stop primitive; the Insert result remains SUCCESS with completed controlled stop.

Test overlays in `config/{fake,gazebo,mujoco}_force_test.yaml` are calibrated only
for the 2000 N/m spring proxy. The legacy spring fixture retains GazeboSystem/JGP; the matched fixture uses
the dynamics-aware torque hardware and exact CAD contact adapter. MuJoCo's test bridge
is a topic-based joint-position plant, not a ros2_control plugin. Native FAIRINO
force APIs are not used. The default is disabled and the node rejects force
activation with the FAIRINO hardware plugin. Only the independently verified
Docker SimMachine transport proof permits a labeled virtual CAD source; real FT
commissioning remains locked.
See [sensor handoff](../../../docs/fr10_force_sensor_handoff.md) and
[recorded evidence](../../../test_results/fr10_force_control_20261001T035857Z/REPORT.md)
for gaps and reproduction commands.

## Local GenericSystem smoke environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fr10_pose_controller fr10_pose_controller.launch.py \
  use_fake_hardware:=true use_rviz:=true
```

The node initializes from simulated joint feedback and remains IDLE until a
valid command arrives. This is not a runtime/integration acceptance target.

## Docker SimMachine / physical FR10

Before launch, the operator must clear hardware faults, select Auto mode, and
enable the robot using the cabinet/vendor controls. The driver verifies these
conditions and does not call `ResetAllError`, `Mode(0)`, or `RobotEnable(1)`.
After the vendor RPC connection is established, the hardware plugin locates
the live TCP descriptor for the configured robot peer, enables and verifies
`TCP_NODELAY`, and fails activation if it cannot prove the setting. The
existing port-8080 readiness probe and native 8 ms `ServoJ` route are retained.

The driver supplies every ServoJ argument explicitly: joint position, zeroed
external axes, `acc=0`, `vel=0`, `cmdT=0.008`, `filterT=0`, and `gain=0`
(`id` uses the SDK default). In the bundled FAIRINO SDK 2.3.3 header,
`acc`, `vel`, `filterT`, and `gain` are marked not open and default to zero;
they are not user PID gains. Consequently Ruckig velocity and acceleration are
trajectory-state/continuity data, while this vendor ServoJ interface consumes
the dense position samples. Do not add an outer PID around ServoJ without a
vendor-supported velocity/torque interface and a separate stability study.

Runtime/integration acceptance explicitly selects
`fairino_hardware_v3_9_3/FairinoHardwareInterface`. ServoCart source and its
standalone benchmark are not built or installed in the production workspace.
Historical comparison evidence is retained under `test_results/`, but the only
current execution path is complete-path ContinuousIK/time-parameterization
through ServoJ.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fr10_pose_controller fr10_pose_controller.launch.py \
  use_fake_hardware:=false use_rviz:=false robot_ip:=192.168.58.2 \
  hardware_plugin:=fairino_hardware_v3_9_3/FairinoHardwareInterface
```

The launch synchronizes commands to the measured joint state and remains IDLE
until an explicit command arrives. Runtime motion uses the FREE_SPACE and
INSERTION scales in `controller.yaml` directly. Keep access to the physical
emergency stop; software controlled stopping is not a certified E-stop.

## Current scope and known gaps

This change completes Joint mode, the common MotionExecutor, eight-branch
analytical IK with independent model cross-checking, dense Cartesian IK
validation, bounded kinematic singularity-detour search for free-space Pose
commands, complete-path Cartesian-aware Pose execution through joint commands,
planned/runtime FK trace checks, derivative-preserving Pose
trajectory boundaries, configurable pre-insert geometry, autonomous
FREE_SPACE-to-INSERTION policy transition from one Pose message, dynamic
PlanningScene snapshot/revision validation, driver health telemetry, and
strict linear insert/retract commands. A verified physical resistance/contact
signal is
still required for contact-aware mating acceptance. The insertion segment
remains straight and never reuses a curved free-space detour. Remaining work
includes physical precision/load characterization and confirmation with
FAIRINO that the bundled SDK's documented ServoJ `cmdT` recommendation applies
unchanged to controller firmware used on the real robot. Shared force simulation
is implemented; real force commissioning and matched CAD contact validation
remain unavailable and cannot be replaced by synthetic-wrench tests.

See the [master specification](../../doc/MASTER_SYSTEM_SPECIFICATION.md).
