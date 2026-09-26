# FR10 controller contribution rules

Read [README.md](README.md),
[controller.yaml](config/controller.yaml), and the
[master specification](../../doc/MASTER_SYSTEM_SPECIFICATION.md) before editing
motion code.

## Non-negotiable architecture

- Never add MoveIt Servo or unvalidated production `ServoCart` actuation. The
  v3.9.3 SDK-level SimMachine benchmark may evaluate ServoCart without
  exporting it as a production command interface.
- Pose input is dense Cartesian path/continuous seeded IK validation ->
  Cartesian-aware full joint path -> joint-only MotionExecutor ->
  `ros2_control`.
- Joint and Pose retargets preserve live position, velocity, and acceleration.
- Never treat stream silence as goal cancellation; the last valid Pose must run
  to completion and HOLD.
- The physical `ros2_control` hardware plugin is the sole actuator owner.
- Do not auto-enable, auto-switch cabinet mode, or auto-reset alarms.
- Keep MoveIt/network/planning calls outside the 125 Hz tick.
- Keep `libfr10_motion.so` free of ROS and MoveIt types.
- Shared conversions belong in `math_utils.hpp`; limits belong in YAML/URDF.

## Change discipline

Preserve verified SDK calls, especially the port 8080 readiness probe and the
native eight-parameter `ServoJ` call with an 8 ms period. Avoid unrelated
refactors. Add a deterministic test for each new core behavior, build both
`fairino_hardware_v3_9_3` and `fr10_pose_controller` when their boundary
changes, and run CTest before handoff. The v3.9.3 plugin is the only supported
FAIRINO runtime backend; do not link a second FAIRINO SDK into this package.

Treat configured singularity/workspace values as engineering barriers, not
certified safety values. Call normal deceleration a controlled stop, not a
hardware emergency stop.
