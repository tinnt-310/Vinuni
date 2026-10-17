#!/usr/bin/env python3
"""Reproducible ROS Insert action test on the canonical MuJoCo FR10 plant.

Start only the topic-based simulator and motion node, in an isolated DDS domain.
No cabinet or ros2_control hardware instance is started by this runner.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from fr10_motion_interfaces.action import Insert
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

ROOT = Path(__file__).resolve().parents[4]
STATE_NAMES = [
    "IDLE", "MOVE_PREINSERT", "HOLD_PREINSERT", "FT_ZERO", "APPROACH",
    "CONTACT_DETECTED", "FORCE_INSERTION", "VERIFY_INSERTION", "SUCCESS",
    "HOLD", "RETRACT", "STOPPING", "FAILED",
]


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [json_safe(v) for v in value]
    return value


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), RMW_LOCALHOST_ONLY="1",
               ROS_LOG_DIR=str(out / "ros_log"))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), RMW_LOCALHOST_ONLY="1",
                      ROS_LOG_DIR=str(out / "client_log"))
    commands = [
        ["python3", "src/vehicle_bringup/test/integration/fr10_mujoco_bridge.py"],
        ["ros2", "launch", "fr10_pose_controller",
         "fr10_pose_controller.launch.py", "start_robot_stack:=false",
         "use_gazebo:=true", "use_rviz:=false", "gui:=false",
         "controller_overlay:=" + str(
             ROOT / "src/fr10_arm_control/fr10_pose_controller/config/"
             "mujoco_force_test.yaml")],
    ]
    if args.backend == 'fake':
        commands = [
            ['python3', 'src/vehicle_bringup/test/integration/fr10_fake_wrench_bridge.py'],
            ['ros2', 'launch', 'fr10_pose_controller', 'fr10_pose_controller.launch.py',
             'use_fake_hardware:=true', 'use_gazebo:=false', 'use_rviz:=false',
             'controller_overlay:='+str(ROOT/'src/fr10_arm_control/fr10_pose_controller/config/fake_force_test.yaml')]]
    if args.backend == "gazebo":
        commands = [
            ["python3", "src/vehicle_bringup/test/integration/fr10_gazebo_force_evidence.py"],
            ["python3", "src/vehicle_bringup/test/integration/fr10_wrench_fault_adapter.py"],
            ["ros2", "launch", "fr10_pose_controller", "fr10_pose_controller.launch.py",
             "use_gazebo:=true", "enable_force_sensor:=true", "use_rviz:=false", "gui:=false",
             "gazebo_controller_start_delay:=0.0",
             "ft_wrench_topic:=raw_wrench",
             "gazebo_world:=" + str(ROOT / "src/fr10_arm_control/fairino10_v6_moveit2_config/worlds/fr10_force_home_test.world"),
             "gazebo_clock_params:=" + str(ROOT / "src/fr10_arm_control/fr10_pose_controller/config/gazebo_force_clock.yaml"),
             "controller_overlay:=" + str(ROOT / "src/fr10_arm_control/fr10_pose_controller/config/gazebo_force_test.yaml")],
        ]
        env.update(GAZEBO_MASTER_URI=f"http://127.0.0.1:{args.port}", GAZEBO_MODEL_DATABASE_URI="")
    processes = []
    files = []
    summary = {"scenario": args.scenario, "commands": commands,
               "domain": args.domain, "desired_force_n": args.force,
               "insertion_depth_m": args.depth,
               "physics": ("none; deterministic spring wrench and canonical FK only" if args.backend=='fake'
                           else "canonical FR10 + CCS2 visual + compliant proxy"),
               "real_robot_contact": False}
    summary["backend"] = args.backend
    summary["source_sha256"] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [ROOT / "src/vehicle_bringup/test/integration/fr10_mujoco_bridge.py",
                     ROOT / "src/vehicle_bringup/test/integration/fr10_gazebo_force_evidence.py",
                     ROOT / "src/fr10_arm_control/fr10_pose_controller/config/controller.yaml",
                     ROOT / f"src/fr10_arm_control/fr10_pose_controller/config/{args.backend}_force_test.yaml"]}
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('fr10_pose_controller')) / 'lib/fr10_pose_controller/fr10_pose_controller_node'
    summary['controller_binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    rclpy.init()
    node = rclpy.create_node("fr10_force_mujoco_test")
    contact = [None]
    actual = [None]
    controller_pose = [None]
    controller_status = [""]
    measured_wrench = [None]
    counts = Counter()
    state_counts = Counter()
    filtered_frames = set()
    correction_frames = set()
    tf_frames = set()
    trace = (out / "trace.jsonl").open("w")
    last_state = [-1]
    fault_sent = [False]
    pause_sent_s = [None]
    pause_snapshot = [None]
    pause_resumed = [False]
    latest_correction = [None]
    latest_command = [None]
    goal_handle = [None]
    fault_pub = node.create_publisher(String, "/fr10_mujoco/wrench_fault", 1)
    scene_pub = node.create_publisher(CollisionObject, "/collision_object", 1)
    from nav_msgs.msg import Odometry
    base_pub = node.create_publisher(Odometry, '/odom', 1)
    started = time.monotonic()

    def record(kind, fields):
        counts[kind] += 1
        trace.write(json.dumps(json_safe(dict(wall_s=time.monotonic() - started,
                                    kind=kind, **fields)), allow_nan=False) + "\n")

    def pose_fields(message):
        pose = message.pose if hasattr(message, 'pose') else message
        p, q = pose.position, pose.orientation
        return {"position": [p.x, p.y, p.z],
                "quaternion": [q.x, q.y, q.z, q.w]}

    def on_actual(message):
        actual[0] = message
        record("actual", pose_fields(message))

    def on_wrench(message):
        w = message.wrench
        measured_wrench[0] = [w.force.x, w.force.y, w.force.z,
                              w.torque.x, w.torque.y, w.torque.z]
        record("wrench", {"values": measured_wrench[0],
                          "source_s": message.header.stamp.sec +
                          message.header.stamp.nanosec * 1e-9})

    def on_joint(message):
        record("joint", {"q": list(message.position),
                         "qd": list(message.velocity),
                         "effort": list(message.effort)})

    def on_command(message):
        latest_command[0] = list(message.data)
        record("command", {"q": list(message.data)})

    def on_joint_reference(message):
        record('joint_reference', {'q': list(message.data[:6]),
            'qd': list(message.data[6:12]), 'qdd': list(message.data[12:18]),
            'acknowledged_command_sequence': message.layout.data_offset})

    def on_filtered(message):
        filtered_frames.add(message.header.frame_id)
        record('filtered_wrench', {'frame': message.header.frame_id,
            'source_s': message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
            'values': [message.wrench.force.x, message.wrench.force.y,
                       message.wrench.force.z, message.wrench.torque.x,
                       message.wrench.torque.y, message.wrench.torque.z]})

    def on_transform(message):
        for frame in message.transforms:
            if frame.child_frame_id.startswith('fr10_force/insertion_axes_at_tcp_'):
                q = frame.transform.rotation
                if frame.header.frame_id == 'fr10_base_link' and abs(
                        sum(v*v for v in (q.x,q.y,q.z,q.w))-1) < 1e-6:
                    tf_frames.add(frame.child_frame_id)
                record('insertion_tf', {'frame': frame.child_frame_id,
                    'parent': frame.header.frame_id,
                    'translation': [frame.transform.translation.x,
                                    frame.transform.translation.y,
                                    frame.transform.translation.z],
                    'quaternion': [q.x,q.y,q.z,q.w]})

    def on_feedback(message):
        f = message.feedback
        latest_correction[0] = list(f.correction)
        state_counts[STATE_NAMES[f.state]] += 1
        if f.state != last_state[0]:
            print(f"STATE {STATE_NAMES[f.state]} depth={f.depth:.6f} "
                  f"fz={f.measured_wrench.force.z:.3f} "
                  f"correction={f.correction[2]:.6f}", flush=True)
            last_state[0] = f.state
        record("force", {"state": f.state, "depth": f.depth,
                         "progress": f.progress,
                         "fz": f.measured_wrench.force.z,
                         "force_error": f.force_error.force.z,
                         "correction": list(f.correction),
                         "contact_state": f.contact_state,
                         "generation": f.generation,
                         "nominal_pose": pose_fields(f.nominal_pose),
                         "corrected_pose": pose_fields(f.corrected_pose),
                         "source_age": f.source_age})
        if (args.scenario not in ("normal", "obstruction", "missing_fixture", "no_contact", "timeout") and not fault_sent[0] and
                f.state == Insert.Feedback.FORCE_INSERTION):
            if args.scenario == "cancel":
                goal_handle[0].cancel_goal_async()
            elif args.scenario == 'clock_jump' and args.backend == 'gazebo':
                from std_srvs.srv import Empty
                reset_clock = node.create_client(Empty,'/reset_simulation')
                if not reset_clock.wait_for_service(timeout_sec=2): raise RuntimeError('reset simulation service unavailable')
                reset_clock.call_async(Empty.Request())
            elif args.scenario == 'pause':
                if args.backend == 'gazebo':
                    from std_srvs.srv import Empty
                    client_pause = node.create_client(Empty,'/pause_physics')
                    if not client_pause.wait_for_service(timeout_sec=2): raise RuntimeError('pause service unavailable')
                    client_pause.call_async(Empty.Request())
                elif args.backend == 'mujoco':
                    fault_pub.publish(String(data='pause'))
                else: raise RuntimeError('pause scenario needs simulation/source time backend')
                pause_sent_s[0] = time.monotonic()
            elif args.scenario == "scene_change":
                change = CollisionObject()
                change.header.frame_id = 'fr10_base_link'
                change.id = 'unexpected_scene_revision'
                change.operation = CollisionObject.ADD
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [0.01, 0.01, 0.01]
                location = Pose()
                location.position.x = 5.0
                location.orientation.w = 1.0
                change.primitives = [primitive]
                change.primitive_poses = [location]
                scene_pub.publish(change)
            elif args.scenario == "base_motion":
                moving = Odometry()
                moving.header.stamp.sec = 1
                moving.twist.twist.linear.x = 0.1
                base_pub.publish(moving)
            else:
                fault_pub.publish(String(data=args.scenario))
            fault_sent[0] = True

    node.create_subscription(PoseStamped, f"/fr10_{args.backend}/contact_pose",
                             lambda m: contact.__setitem__(0, m), 1)
    node.create_subscription(PoseStamped, f"/fr10_{args.backend}/tcp_pose", on_actual,
                             qos_profile_sensor_data)
    node.create_subscription(PoseStamped, "/fr10_motion/tcp_pose",
                             lambda m: controller_pose.__setitem__(0, m),
                             qos_profile_sensor_data)
    node.create_subscription(String, '/controller_status',
                             lambda m: controller_status.__setitem__(0, m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(WrenchStamped, "/ft_sensor/wrench", on_wrench,
                             qos_profile_sensor_data)
    node.create_subscription(WrenchStamped, '/fr10_force/filtered_wrench',
                             on_filtered, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, '/fr10_force/correction',
        lambda m: correction_frames.add(m.header.frame_id), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', on_transform, 100)
    node.create_subscription(JointState, "/joint_states", on_joint,
                             qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray,
                             "/fairino10_controller/commands", on_command, 10)
    node.create_subscription(Float64MultiArray, '/fr10_force/joint_reference',
                             on_joint_reference, qos_profile_sensor_data)
    client = ActionClient(node, Insert, "/fr10_force/insert")
    try:
        names = ('plant', 'relay', 'controller') if args.backend == 'gazebo' else ('plant', 'controller')
        for name, command in zip(names, commands):
            log = (out / (name + ".log")).open("w")
            files.append(log)
            processes.append(subprocess.Popen(
                command, cwd=ROOT, env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("simulator/controller exited during startup")
            initialized = False
            if actual[0] and controller_pose[0]:
                a = actual[0].pose.position
                b = controller_pose[0].pose.position
                initialized = math.dist((a.x, a.y, a.z),
                                        (b.x, b.y, b.z)) < 0.002
            if (contact[0] and initialized and client.server_is_ready() and
                    'backend_ready=true' in controller_status[0]):
                break
        else:
            raise RuntimeError("MuJoCo/action startup deadline exceeded")
        # A measured controller TCP agreeing with simulator truth proves the
        # first feedback tick; action discovery alone races initialization.
        goal = Insert.Goal()
        summary['startup_controller_status'] = controller_status[0]
        goal.target_pose = contact[0]
        goal.preinsert_distance = 0.005
        goal.insertion_depth = args.depth
        goal.insertion_speed = 0.002
        goal.desired_wrench.force.z = args.force
        goal.selected_axes = [False, False, True, False, False, False]
        goal.max_force = [50.0, 50.0, 50.0]
        goal.max_torque = [10.0, 10.0, 10.0]
        goal.timeout.sec = 1 if args.scenario == 'timeout' else args.timeout
        goal.profile_id = {'mujoco':'mujoco_v1','gazebo':'simulation_v1','fake':'fake_v1'}[args.backend]
        goal.completion_behavior = (Insert.Goal.RETRACT if args.retract else Insert.Goal.HOLD)
        q = goal.target_pose.pose.orientation
        axis = [2 * (q.x * q.z + q.w * q.y),
                2 * (q.y * q.z - q.w * q.x),
                1 - 2 * (q.x * q.x + q.y * q.y)]
        p = goal.target_pose.pose.position
        contact_position = [p.x, p.y, p.z]
        p.x += axis[0] * args.depth
        p.y += axis[1] * args.depth
        p.z += axis[2] * args.depth
        summary.update(contact_position=contact_position, axis=axis,
                       target_position=[p.x, p.y, p.z])
        fixture = CollisionObject()
        fixture.header.frame_id = 'fr10_base_link'
        fixture.id = 'force_contact_proxy'
        fixture.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.30, 0.30, 0.01]
        fixture_pose = Pose()
        fixture_pose.orientation = q
        fixture_pose.position.x = contact_position[0] + axis[0]*0.005
        fixture_pose.position.y = contact_position[1] + axis[1]*0.005
        fixture_pose.position.z = contact_position[2] + axis[2]*0.005
        fixture.primitives = [box]
        fixture.primitive_poses = [fixture_pose]
        while not scene_pub.get_subscription_count():
            if time.monotonic() > deadline: raise RuntimeError('PlanningScene subscriber unavailable')
            rclpy.spin_once(node, timeout_sec=0.02)
        if args.scenario != 'missing_fixture': scene_pub.publish(fixture)
        if args.scenario == 'no_contact':
            fault_pub.publish(String(data='lost_contact'))
            fault_sent[0] = True
        if args.scenario == 'obstruction':
            fixture.id = 'forbidden_obstruction'
            scene_pub.publish(fixture)
            fault_sent[0] = True
        until = time.monotonic() + 0.25
        while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=0.01)
        future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        goal_handle[0] = future.result()
        if goal_handle[0] is None or not goal_handle[0].accepted:
            raise RuntimeError("Insert goal was rejected")
        result_future = goal_handle[0].get_result_async()
        deadline = time.monotonic() + args.timeout + 20
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if pause_sent_s[0] and not pause_resumed[0]:
                elapsed = time.monotonic()-pause_sent_s[0]
                if elapsed > 0.2 and pause_snapshot[0] is None:
                    pause_snapshot[0] = (counts['command'],pose_fields(actual[0]),latest_correction[0])
                if elapsed > 1.2:
                    snapshot = pause_snapshot[0]
                    summary['pause_hold_verified'] = (snapshot is not None and
                        counts['command'] == snapshot[0] and pose_fields(actual[0]) == snapshot[1] and
                        latest_correction[0] == snapshot[2])
                    if args.backend == 'gazebo':
                        from std_srvs.srv import Empty
                        client_resume = node.create_client(Empty,'/unpause_physics')
                        if not client_resume.wait_for_service(timeout_sec=2): raise RuntimeError('resume service unavailable')
                        client_resume.call_async(Empty.Request())
                    else: fault_pub.publish(String(data='normal'))
                    pause_resumed[0] = True
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("simulator/controller exited during action")
        if not result_future.done():
            raise RuntimeError("Insert result deadline exceeded")
        result = result_future.result().result
        if result.success:
            held_command = latest_command[0]
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('plant/controller exited during post-result HOLD')
            summary['post_result_hold_verified'] = (
                held_command is not None and latest_command[0] is not None and
                max(abs(a-b) for a,b in zip(held_command,latest_command[0])) < 1e-8 and
                'state=HOLDING' in controller_status[0])
        final_pose = actual[0].pose.position
        actual_depth = sum((value - origin) * direction for value, origin,
                           direction in zip(
                               (final_pose.x, final_pose.y, final_pose.z),
                               contact_position, axis))
        summary.update(success=result.success, code=result.code,
                       message=result.message, final_depth=result.final_depth,
                       independent_actual_depth=actual_depth,
                       peak_force=result.peak_force, peak_torque=result.peak_torque,
                       controlled_stop_completed=result.controlled_stop_completed,
                       final_wrench=measured_wrench[0], fault_injected=fault_sent[0])
        summary['final_controller_status'] = controller_status[0]
        expected = {"normal": 0, "pause": 0, "clock_jump":20, "no_contact":18, "timeout":18,
                    "dropout": 5, "duplicate": 5, "nan": 4,
                    "overload": 6, "torque": 7, "lost_contact": 11,
                    "cancel": 19, "scene_change": 14,
                    "base_motion": 15,
                    "obstruction": 3, "missing_fixture": 3}[args.scenario]
        summary["pass"] = result.code == expected and (
            (result.success and (abs(actual_depth + 0.005) <= 0.0005 if args.retract
                                 else abs(actual_depth - args.depth) <= 0.0005))
            if args.scenario in ('normal','pause') else
            (not result.success and fault_sent[0]) if args.scenario=='clock_jump' else
            (not result.success) if args.scenario in ('obstruction', 'missing_fixture') else
            (not result.success and result.controlled_stop_completed and
             (fault_sent[0] or args.scenario=='timeout')))
        if args.scenario == 'pause': summary['pass'] &= summary.get('pause_hold_verified',False)
        summary['conditioned_frame_contract'] = (
            bool(filtered_frames) and filtered_frames == correction_frames and
            filtered_frames <= tf_frames)
        summary['conditioned_frames'] = sorted(filtered_frames)
        if args.scenario in ('normal', 'pause'):
            summary['pass'] &= summary['conditioned_frame_contract']
            summary['pass'] &= summary.get('post_result_hold_verified',False)
    except Exception as error:
        summary.update(pass_=False, error=str(error))
        summary["pass"] = False
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        for log in files:
            log.close()
        trace.close()
        summary.update(wall_duration_s=time.monotonic() - started,
                       samples=dict(counts), states=dict(state_counts),
                       process_returncodes=[p.returncode for p in processes])
        node.destroy_node()
        rclpy.shutdown()
        (out / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, allow_nan=False) + "\n")
        print(json.dumps(json_safe(summary), indent=2, allow_nan=False), flush=True)
    return 0 if summary.get("pass") else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "test_results" /
                        ("fr10_force_mujoco_" + datetime.now(timezone.utc).
                         strftime("%Y%m%dT%H%M%SZ")))
    parser.add_argument("--domain", type=int, default=191)
    parser.add_argument("--backend", choices=["mujoco", "gazebo", "fake"], default="mujoco")
    parser.add_argument("--port", type=int, default=11997)
    parser.add_argument("--scenario", default="normal", choices=[
        "normal", "dropout", "duplicate", "nan", "overload", "torque",
        "lost_contact", "cancel", "scene_change", "obstruction", "missing_fixture", "base_motion",
        "pause", "clock_jump", "no_contact", "timeout"])
    parser.add_argument("--force", type=float, default=10.0)
    parser.add_argument("--retract", action="store_true", help="Verify requested success RETRACT instead of HOLD")
    parser.add_argument("--depth", type=float, default=0.005)
    parser.add_argument("--timeout", type=int, default=30)
    raise SystemExit(run(parser.parse_args()))
