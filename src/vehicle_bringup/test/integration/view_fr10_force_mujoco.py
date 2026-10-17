#!/usr/bin/env python3
"""Display measured joints live or replay a recording. No ROS commands.

--run-directory displays the matched CAD from an active or captured trial.
--trace retains the older compliant proxy viewer for its own trace format.
Displayed poses use measured joints; forces are not re-simulated here.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from fr10_mujoco_bridge import Fr10MujocoPlant, JOINT_NAMES


def configure_rendering(option, matched):
    # Group 0: contact cells; 1: fixture; 2: original visual CAD.
    option.geomgroup[0] = 0 if matched else 1
    option.geomgroup[1] = 1
    option.geomgroup[2] = 1
    option.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
    if matched:
        option.flags[mujoco.mjtVisFlag.mjVIS_FLEXFACE] = True


def load_frames(path: Path) -> list[dict]:
    frames = []
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if row.get("kind") == "joint":
            q = np.asarray(row["q"], dtype=float)
            if q.shape != (6,) or not np.all(np.isfinite(q)):
                raise ValueError("trace contains an invalid joint sample")
            frames.append({"time": float(row["wall_s"]), "q": q})
    if not frames or any(b["time"] < a["time"]
                         for a, b in zip(frames, frames[1:])):
        raise ValueError("trace must contain chronological joint samples")
    origin = frames[0]["time"]
    for frame in frames:
        frame["time"] -= origin
    return frames


def load_run_frames(directory: Path) -> list[dict]:
    rows = json.loads((directory / 'joint_feedback.json').read_text())
    if not rows:
        raise ValueError('run contains no measured joint samples')
    frames = []
    origin = float(rows[0]['wall'])
    for row in rows:
        q = np.asarray(row['position'], dtype=float)
        at = float(row['wall']) - origin
        if q.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(at) or at < 0:
            raise ValueError('run contains invalid measured joint samples')
        if frames and at < frames[-1]['time']:
            raise ValueError('run contains backward wall timestamps')
        frames.append({'time': at, 'q': q})
    return frames


@contextmanager
def live_joints(enabled):
    if not enabled:
        yield None
        return
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('fr10_mujoco_read_only_view')
    sample = {'q': None, 'stamp': -1, 'receipt': 0., 'node': node, 'generation': 0}

    def receive(message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if (len(message.name) != 6 or set(message.name) != set(JOINT_NAMES) or
                len(message.position) != 6 or stamp <= 0):
            return
        q = np.array([message.position[message.name.index(name)] for name in JOINT_NAMES])
        if not np.isfinite(q).all():
            return
        # Sequential trials reuse the ROS domain but restart simulation time.
        # Only accept a backward epoch after an actual receipt gap; duplicates
        # from a frozen publisher remain stale.
        if stamp < sample['stamp'] and time.monotonic() - sample['receipt'] > .05:
            sample['stamp'] = -1
            sample['generation'] += 1
        if stamp <= sample['stamp']:
            return
        sample.update(q=q, stamp=stamp, receipt=time.monotonic())

    node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
    try:
        yield sample
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--trace", type=Path)
    source.add_argument("--run-directory", type=Path)
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--close-up", action="store_true")
    parser.add_argument('--live', action='store_true', help='Observe /joint_states in the current ROS domain')
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("speed must be positive and finite")
    matched = args.run_directory is not None
    if args.live and not matched:
        parser.error('--live requires --run-directory for the physical socket model')
    frames = [] if args.live else load_run_frames(args.run_directory) if matched else load_frames(args.trace)
    plant = Fr10MujocoPlant(station_file=args.run_directory / 'selected_station.json',
                           include_visuals=True) if matched else Fr10MujocoPlant()
    caption = 'Matched CAD: recorded measured joints' if matched else 'Compliant proxy: recorded joints'
    verdict = ''
    if matched:
        if not args.live:
            summary = json.loads((args.run_directory / 'summary.json').read_text())
            verdict = f"Capture: {summary.get('force_self_alignment_gate')} | Physical fit: {summary.get('physical_seating_gate')}"
        else:
            caption = 'Matched CAD: live measured joints'
        # Display the exact socket flex faces distinctly from the black plug.
        # These rendering properties do not alter contact geometry or forces.
        plant.model.flex_rgba[:] = [.15, .5, .85, 1.]
        plant.model.vis.headlight.ambient[:] = .6
        plant.model.vis.headlight.diffuse[:] = .7
    paused = False
    close_up = args.close_up
    reset_camera = True

    def key_callback(key: int) -> None:
        nonlocal paused, close_up, reset_camera
        if key == 32 and not args.live:
            paused = not paused
        elif key in (ord("C"), ord("c")):
            close_up = not close_up
            reset_camera = True

    with live_joints(args.live) as live, mujoco.viewer.launch_passive(
            plant.model, plant.data, key_callback=key_callback) as viewer:
        configure_rendering(viewer.opt, matched)
        start = time.monotonic()
        previous = start
        replay_time = 0.0
        index = 0
        print(f"Viewer ready: {args.run_directory or args.trace}; {'live read-only joints' if args.live else 'recorded joint replay'}", flush=True)
        while viewer.is_running():
            now = time.monotonic()
            elapsed = min(now - previous, 0.1)
            previous = now
            if live is not None:
                import rclpy
                rclpy.spin_once(live['node'], timeout_sec=0.)
                if live['q'] is not None:
                    plant.data.qpos[:6] = live['q']
                age = time.monotonic() - live['receipt']
                verdict = f"Receipt age {age:.3f} s | epoch {live['generation']}" if live['q'] is not None else 'Waiting for measured joints'
                if age > .05 and live['q'] is not None:
                    verdict += ' | STALE'
            else:
                if not paused:
                    replay_time += elapsed * args.speed
                if replay_time > frames[-1]["time"] + 3.0:
                    replay_time, index = 0.0, 0
                while index + 1 < len(frames) and frames[index + 1]["time"] <= replay_time:
                    index += 1
                plant.data.qpos[:6] = frames[index]["q"]
            plant.data.qvel[:] = 0.0
            mujoco.mj_forward(plant.model, plant.data)
            tcp, _ = plant.tcp_pose()
            depth = float(np.dot(tcp - plant.contact_tcp_position,
                                 plant.insertion_axis_world))
            if not matched:
                plant.data.qpos[6] = max(0.0, depth)
                mujoco.mj_forward(plant.model, plant.data)
            with viewer.lock():
                if reset_camera:
                    viewer.cam.lookat[:] = (plant.contact_tcp_position if close_up
                                            else np.array([-0.4, -0.1, 0.45]))
                    viewer.cam.distance = (.45 if matched else .38) if close_up else 2.1
                    viewer.cam.azimuth = 135
                    viewer.cam.elevation = -20
                    reset_camera = False
            viewer.set_texts([
                (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 "FR10: MuJoCo measured joints",
                 f"Depth {depth * 1000:.2f} mm | " + ('LIVE' if args.live else f'replay {args.speed:g}x') + '\n' +
                 ('' if args.live else 'Space: pause | ') + 'C: full arm / TCP close-up'),
                (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 caption,
                 (verdict + '\n' if matched else 'Proxy contact; mating not validated\n') +
                 ('Live view; no ROS publisher.' if args.live else 'Recorded joint replay; no ROS publisher.')),
            ])
            viewer.sync()
            time.sleep(1.0 / 60.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
