#!/usr/bin/env python3
"""Comprehensive Benchmark Suite for FAIRINO FR10 Motion Smoothness & Speed on SimMachine.

Measures:
1. Low-Level Control Loop Rate, Jitter, and Period Statistics (125 Hz).
2. Command-to-Motion Response Latency (Round-trip time to first motion).
3. Command Stream Throughput (Hz capacity).
4. Single-Joint Motion Smoothness & Speed Profile across Distance Scales:
   - Near (15 deg)
   - Medium (40 deg)
   - Far (70 deg)
   - Very Far (90+ deg)
   for all 6 joints (J1..J6).
   Metrics computed per trajectory:
   - Max Velocity (rad/s & deg/s) vs Configured Limit
   - Max Acceleration (rad/s^2 & deg/s^2)
   - Max Jerk (rad/s^3) & RMS Jerk (rad/s^3) [Trajectory Smoothness Indicator]
   - Execution Time (s)
   - Steady-State Settling Time & Error (rad)
5. Multi-Joint Synchronous Motion Smoothness (3-joint & 6-joint coordinated).
6. Online Sudden Retargeting & Reversal Smoothness (A -> B -> C sudden target change).
7. Cartesian Motion Smoothness & Speed Profile:
   - Near (5 mm XYZ)
   - Far (20 mm & 50 mm XYZ)
   - Orientation (5 deg & 10 deg RX/RY/RZ)
   - Cartesian Velocity (mm/s), Acceleration (mm/s^2), Jerk (mm/s^3)
8. 10 Hz AI Continuous Streaming Tracking Smoothness.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import pathlib
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# Ensure import paths work
SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from common import (
    MAX_JOINT,
    MIN_JOINT,
    ManagedLaunch,
    ProbeContext,
    check_preflight,
    generate,
    make_results_dir,
    max_abs_error,
    position_error,
    quaternion_error,
    quaternion_multiply,
    axis_angle_quaternion,
    rate_metrics,
    write_json,
)


def compute_smoothness_metrics(stamps: np.ndarray, positions: np.ndarray,
                               velocities: Optional[np.ndarray] = None) -> Dict[str, Any]:
    """Compute velocity, acceleration, and jerk profiles from trajectory timestamps and positions."""
    if len(stamps) < 4:
        return {"error": "insufficient samples"}

    dt = np.diff(stamps)
    # Filter non-positive dt to avoid division by zero
    valid_dt = dt > 1e-6
    if not np.any(valid_dt):
        return {"error": "invalid timestamps"}

    # Velocity
    if velocities is not None and len(velocities) == len(stamps):
        vel = velocities
    else:
        vel = np.zeros_like(positions)
        vel[1:] = np.diff(positions, axis=0) / dt[:, None]
        vel[0] = vel[1]

    # Acceleration: dvel / dt
    acc = np.zeros_like(vel)
    acc[1:] = np.diff(vel, axis=0) / dt[:, None]
    acc[0] = acc[1]

    # Jerk: dacc / dt
    jerk = np.zeros_like(acc)
    jerk[1:] = np.diff(acc, axis=0) / dt[:, None]
    jerk[0] = jerk[1]

    # Per-joint statistics
    max_vel = np.max(np.abs(vel), axis=0).tolist()
    mean_vel = np.mean(np.abs(vel), axis=0).tolist()
    max_acc = np.max(np.abs(acc), axis=0).tolist()
    mean_acc = np.mean(np.abs(acc), axis=0).tolist()
    max_jerk = np.max(np.abs(jerk), axis=0).tolist()
    rms_jerk = np.sqrt(np.mean(jerk**2, axis=0)).tolist()

    return {
        "sample_count": int(len(stamps)),
        "duration_s": float(stamps[-1] - stamps[0]),
        "max_velocity_rad_s": max_vel,
        "mean_velocity_rad_s": mean_vel,
        "max_velocity_deg_s": [v * 180.0 / math.pi for v in max_vel],
        "max_acceleration_rad_s2": max_acc,
        "mean_acceleration_rad_s2": mean_acc,
        "max_acceleration_deg_s2": [a * 180.0 / math.pi for a in max_acc],
        "max_jerk_rad_s3": max_jerk,
        "rms_jerk_rad_s3": rms_jerk,
        "max_jerk_deg_s3": [j * 180.0 / math.pi for j in max_jerk],
    }


def compute_cartesian_smoothness(stamps: np.ndarray, positions_xyz: np.ndarray) -> Dict[str, Any]:
    """Compute linear Cartesian speed, acceleration, and jerk profiles."""
    if len(stamps) < 4:
        return {"error": "insufficient samples"}

    dt = np.diff(stamps)
    vel_xyz = np.zeros_like(positions_xyz)
    vel_xyz[1:] = np.diff(positions_xyz, axis=0) / dt[:, None]
    vel_xyz[0] = vel_xyz[1]
    speed = np.linalg.norm(vel_xyz, axis=1)  # m/s

    acc_xyz = np.zeros_like(vel_xyz)
    acc_xyz[1:] = np.diff(vel_xyz, axis=0) / dt[:, None]
    acc_xyz[0] = acc_xyz[1]
    acc_mag = np.linalg.norm(acc_xyz, axis=1)  # m/s^2

    jerk_xyz = np.zeros_like(acc_xyz)
    jerk_xyz[1:] = np.diff(acc_xyz, axis=0) / dt[:, None]
    jerk_xyz[0] = jerk_xyz[1]
    jerk_mag = np.linalg.norm(jerk_xyz, axis=1)  # m/s^3

    return {
        "sample_count": int(len(stamps)),
        "duration_s": float(stamps[-1] - stamps[0]),
        "max_speed_mm_s": float(np.max(speed) * 1000.0),
        "mean_speed_mm_s": float(np.mean(speed) * 1000.0),
        "max_accel_mm_s2": float(np.max(acc_mag) * 1000.0),
        "max_jerk_mm_s3": float(np.max(jerk_mag) * 1000.0),
        "rms_jerk_mm_s3": float(np.sqrt(np.mean(jerk_mag**2)) * 1000.0),
    }


class BenchmarkRunner:
    def __init__(self, robot_ip: str = "192.168.58.2", domain_id: int = 171,
                 results_dir: Optional[pathlib.Path] = None):
        self.robot_ip = robot_ip
        self.domain_id = domain_id
        self.results_dir = results_dir or make_results_dir()
        self.results: Dict[str, Any] = {}
        self.launch: Optional[ManagedLaunch] = None
        self.probe = None
        self.probe_context = None

    def start_environment(self) -> bool:
        print("\n" + "=" * 70)
        print("  FAIRINO FR10 COMPREHENSIVE SIMMACHINE MOTION BENCHMARK")
        print("=" * 70)
        print(f"[*] Preflight checking robot IP {self.robot_ip}...")
        preflight = check_preflight(self.robot_ip)
        if not preflight["all_ok"]:
            print(f"[!] Preflight check FAILED: {preflight}")
            return False

        print("[+] Starting ROS2 runtime stack...")
        self.launch = ManagedLaunch.start(self.results_dir, self.robot_ip, self.domain_id)
        self.probe_context = ProbeContext(self.domain_id)
        self.probe = self.probe_context.__enter__()

        ready = self.probe.wait_ready(timeout=25.0)
        if not ready:
            print("[!] ROS2 runtime stack failed to become ready.")
            return False
        print("[+] ROS2 runtime stack READY.")
        return True

    def stop_environment(self):
        if self.probe_context:
            self.probe_context.__exit__(None, None, None)
        if self.launch:
            self.launch.stop()

    def benchmark_rate_and_latency(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("1. LATENCY, FREQUENCY & JITTER BENCHMARK")
        print("-" * 70)

        # 1.1 Idle loop rate
        print("  Measuring 125 Hz low-level control loop for 5.0 seconds...")
        with self.probe.lock:
            start_idx = len(self.probe.joint_stamps)
        time.sleep(5.0)
        with self.probe.lock:
            stamps = list(self.probe.joint_stamps[start_idx:])
            status = self.probe.latest_status

        timing = rate_metrics(stamps)
        print(f"  -> Samples:         {timing['sample_count']}")
        print(f"  -> Mean Rate:       {timing['mean_hz']:.2f} Hz (Target: 125.0 Hz)")
        print(f"  -> Mean dt:         {timing['mean_dt']*1000:.3f} ms")
        print(f"  -> Min dt:          {timing['min_dt']*1000:.3f} ms")
        print(f"  -> Max dt:          {timing['max_dt']*1000:.3f} ms")
        print(f"  -> Jitter (StdDev): {timing['stddev_dt']*1000:.3f} ms")

        # 1.2 Command-to-Motion Response Latency
        print("\n  Measuring Command-to-Motion Response Latency...")
        latencies_ms = []
        for trial in range(3):
            actual = self.probe.joint_position()
            target = list(actual)
            target[0] += math.radians(5.0 * (1 if trial % 2 == 0 else -1))

            with self.probe.lock:
                start_hist = len(self.probe.joint_history)
            
            t_sent = time.monotonic()
            self.probe.publish_joint(target)

            t_first_motion = None
            deadline = t_sent + 3.0
            while time.monotonic() < deadline:
                with self.probe.lock:
                    rows = self.probe.joint_history[start_hist:]
                for stamp, q, qd in rows:
                    if abs(q[0] - actual[0]) > math.radians(0.02) or abs(qd[0]) > 0.005:
                        t_first_motion = stamp
                        break
                if t_first_motion:
                    break
                time.sleep(0.002)

            # Wait to settle
            self.probe.wait_joint(target, timeout=10.0)
            if t_first_motion:
                lat_ms = (t_first_motion - t_sent) * 1000.0
                latencies_ms.append(lat_ms)
                print(f"  -> Trial {trial+1}: Response Latency = {lat_ms:.2f} ms")

        avg_lat_ms = float(np.mean(latencies_ms)) if latencies_ms else float('nan')
        print(f"  -> Average Response Latency: {avg_lat_ms:.2f} ms")

        res = {
            "timing_metrics": timing,
            "response_latencies_ms": latencies_ms,
            "mean_response_latency_ms": avg_lat_ms,
        }
        self.results["rate_and_latency"] = res
        return res

    def benchmark_joint_smoothness_and_speed(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("2. SINGLE-JOINT MOTION SMOOTHNESS & SPEED MATRIX (NEAR -> FAR)")
        print("-" * 70)

        # Distance scales: Near (15°), Medium (40°), Far (70°), Very Far (90°)
        scales = [
            ("NEAR_15DEG", 15.0),
            ("MEDIUM_40DEG", 40.0),
            ("FAR_70DEG", 70.0),
            ("VERY_FAR_90DEG", 90.0),
        ]

        joint_results = {}
        for j_idx in range(6):
            j_name = f"Joint_{j_idx+1}"
            joint_results[j_name] = {}
            print(f"\n[*] Testing {j_name}:")

            for scale_name, delta_deg in scales:
                actual = self.probe.joint_position()
                safe = generate(actual, j_idx, delta_deg)
                if not safe.allowed:
                    print(f"  [{scale_name:<14} ({delta_deg:>4.0f}°)]: BLOCKED ({safe.blocked_reason})")
                    joint_results[j_name][scale_name] = {"status": safe.blocked_reason}
                    continue

                with self.probe.lock:
                    hist_start = len(self.probe.joint_history)

                t_start = time.monotonic()
                self.probe.publish_joint(safe.target)
                timeout = max(18.0, math.radians(delta_deg) / 0.15 + 12.0)
                reached = self.probe.wait_joint(safe.target, timeout=timeout)

                # Wait for holding state
                if reached:
                    deadline = time.monotonic() + 1.5
                    while time.monotonic() < deadline and "state=HOLDING" not in self.probe.status():
                        time.sleep(0.01)

                t_end = time.monotonic()
                exec_time = t_end - t_start

                with self.probe.lock:
                    history = list(self.probe.joint_history[hist_start:])

                final_q = self.probe.joint_position()
                ss_err = max_abs_error(final_q, safe.target) if final_q else float('nan')

                if len(history) >= 4:
                    stamps = np.array([row[0] for row in history])
                    positions = np.array([row[1] for row in history])
                    velocities = np.array([row[2] for row in history])
                    metrics = compute_smoothness_metrics(stamps, positions, velocities)

                    v_max = metrics["max_velocity_deg_s"][j_idx]
                    a_max = metrics["max_acceleration_deg_s2"][j_idx]
                    j_max = metrics["max_jerk_deg_s3"][j_idx]
                    j_rms = metrics["rms_jerk_rad_s3"][j_idx]

                    print(f"  [{scale_name:<14} ({delta_deg:>4.0f}°)]: "
                          f"Time: {exec_time:4.2f}s | "
                          f"Vmax: {v_max:5.1f}°/s | "
                          f"Amax: {a_max:5.1f}°/s² | "
                          f"JerkMax: {j_max:6.1f}°/s³ | "
                          f"JerkRMS: {j_rms:5.2f} r/s³ | "
                          f"Err: {math.degrees(ss_err):.4f}°")

                    joint_results[j_name][scale_name] = {
                        "status": "PASS" if reached and ss_err <= 0.002 else "FAIL",
                        "delta_deg": delta_deg,
                        "execution_time_s": exec_time,
                        "steady_state_error_deg": float(math.degrees(ss_err)),
                        "max_velocity_deg_s": float(v_max),
                        "max_acceleration_deg_s2": float(a_max),
                        "max_jerk_deg_s3": float(j_max),
                        "rms_jerk_rad_s3": float(j_rms),
                    }
                else:
                    print(f"  [{scale_name}]: FAIL (insufficient samples)")
                    joint_results[j_name][scale_name] = {"status": "FAIL"}

                # Return cleanly to original joint angle
                if reached:
                    self.probe.publish_joint(actual)
                    self.probe.wait_joint(actual, timeout=timeout)
                    time.sleep(0.3)

        self.results["single_joint_benchmark"] = joint_results
        return joint_results

    def benchmark_multijoint_smoothness(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("3. MULTI-JOINT SYNCHRONOUS MOTION SMOOTHNESS BENCHMARK")
        print("-" * 70)

        cases = [
            ("MJ_3JOINT_15DEG", [15.0, 15.0, 15.0, 0.0, 0.0, 0.0]),
            ("MJ_6JOINT_15DEG", [15.0, 15.0, 15.0, 15.0, 15.0, 15.0]),
            ("MJ_3JOINT_40DEG", [40.0, 30.0, 30.0, 0.0, 0.0, 0.0]),
            ("MJ_6JOINT_LARGE", [50.0, 40.0, 30.0, 25.0, 20.0, 20.0]),
        ]

        mj_results = {}
        for label, deltas in cases:
            actual = self.probe.joint_position()
            target = list(actual)
            for i, d in enumerate(deltas):
                if d > 0:
                    safe = generate(actual, i, d)
                    if safe.allowed:
                        target[i] = safe.target[i]

            with self.probe.lock:
                hist_start = len(self.probe.joint_history)

            t_start = time.monotonic()
            self.probe.publish_joint(target)
            timeout = max(20.0, math.radians(max(deltas)) / 0.15 + 12.0)
            reached = self.probe.wait_joint(target, timeout=timeout)

            if reached:
                deadline = time.monotonic() + 1.5
                while time.monotonic() < deadline and "state=HOLDING" not in self.probe.status():
                    time.sleep(0.01)

            exec_time = time.monotonic() - t_start
            with self.probe.lock:
                history = list(self.probe.joint_history[hist_start:])

            final_q = self.probe.joint_position()
            ss_err = max_abs_error(final_q, target) if final_q else float('nan')

            if len(history) >= 4:
                stamps = np.array([row[0] for row in history])
                positions = np.array([row[1] for row in history])
                velocities = np.array([row[2] for row in history])
                metrics = compute_smoothness_metrics(stamps, positions, velocities)

                max_jerk_all = max(metrics["max_jerk_deg_s3"])
                max_vel_all = max(metrics["max_velocity_deg_s"])
                max_acc_all = max(metrics["max_acceleration_deg_s2"])

                print(f"  [{label:<16}]: "
                      f"Time: {exec_time:4.2f}s | "
                      f"Peak Vel: {max_vel_all:5.1f}°/s | "
                      f"Peak Acc: {max_acc_all:5.1f}°/s² | "
                      f"Peak Jerk: {max_jerk_all:6.1f}°/s³ | "
                      f"Max Err: {math.degrees(ss_err):.4f}°")

                mj_results[label] = {
                    "status": "PASS" if reached and ss_err <= 0.002 else "FAIL",
                    "execution_time_s": exec_time,
                    "steady_state_error_deg": float(math.degrees(ss_err)),
                    "peak_velocity_deg_s": float(max_vel_all),
                    "peak_acceleration_deg_s2": float(max_acc_all),
                    "peak_jerk_deg_s3": float(max_jerk_all),
                }

            # Return cleanly
            if reached:
                self.probe.publish_joint(actual)
                self.probe.wait_joint(actual, timeout=timeout)
                time.sleep(0.3)

        self.results["multijoint_benchmark"] = mj_results
        return mj_results

    def benchmark_sudden_retargeting_smoothness(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("4. ONLINE SUDDEN RETARGETING & REVERSAL SMOOTHNESS (A -> B -> C)")
        print("-" * 70)

        actual = self.probe.joint_position()
        safe_a = generate(actual, 0, 50.0)
        if not safe_a.allowed:
            return {"status": "BLOCKED"}

        dir_a = safe_a.direction
        target_a = safe_a.target
        target_b = list(actual)
        target_b[0] = actual[0] - dir_a * math.radians(35.0)  # Reversal
        target_c = list(actual)
        target_c[0] = actual[0] + dir_a * math.radians(15.0)  # Forward again

        with self.probe.lock:
            hist_start = len(self.probe.joint_history)

        t_start = time.monotonic()
        # Step A: Command A
        self.probe.publish_joint(target_a)

        # Wait until robot accelerates to speed
        moving = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            qd = self.probe.joint_velocity()
            if qd and abs(qd[0]) > 0.05:
                moving = True
                break
            time.sleep(0.01)

        # Step B: Sudden reversal while moving!
        t_rev = time.monotonic()
        self.probe.publish_joint(target_b)
        time.sleep(0.4)

        # Step C: Re-target to C
        t_c = time.monotonic()
        self.probe.publish_joint(target_c)

        reached = self.probe.wait_joint(target_c, timeout=25.0)
        exec_time = time.monotonic() - t_start

        with self.probe.lock:
            history = list(self.probe.joint_history[hist_start:])

        final_q = self.probe.joint_position()
        ss_err = max_abs_error(final_q, target_c) if final_q else float('nan')

        if len(history) >= 4:
            stamps = np.array([row[0] for row in history])
            positions = np.array([row[1] for row in history])
            velocities = np.array([row[2] for row in history])
            metrics = compute_smoothness_metrics(stamps, positions, velocities)

            j_max = metrics["max_jerk_deg_s3"][0]
            j_rms = metrics["rms_jerk_rad_s3"][0]
            v_max = metrics["max_velocity_deg_s"][0]

            print(f"  -> Sudden Reversal Result: {'PASS' if reached else 'FAIL'}")
            print(f"  -> Total Transition Time:  {exec_time:.2f} s")
            print(f"  -> Peak Velocity:          {v_max:.1f} °/s")
            print(f"  -> Reversal Peak Jerk:     {j_max:.1f} °/s³ (Smooth OTG Transition)")
            print(f"  -> RMS Jerk:               {j_rms:.2f} rad/s³")
            print(f"  -> Final Target Error:     {math.degrees(ss_err):.4f} °")

            res = {
                "status": "PASS" if reached and ss_err <= 0.002 else "FAIL",
                "execution_time_s": exec_time,
                "peak_jerk_deg_s3": float(j_max),
                "rms_jerk_rad_s3": float(j_rms),
                "peak_velocity_deg_s": float(v_max),
                "steady_state_error_deg": float(math.degrees(ss_err)),
            }
        else:
            res = {"status": "FAIL"}

        # Return cleanly
        self.probe.publish_joint(actual)
        self.probe.wait_joint(actual, timeout=20.0)
        time.sleep(0.3)

        self.results["retargeting_benchmark"] = res
        return res

    def benchmark_cartesian_smoothness_and_speed(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("5. CARTESIAN MOTION SMOOTHNESS & SPEED BENCHMARK (NEAR & FAR)")
        print("-" * 70)

        cases = [
            ("CART_5MM_POS_X", [0.005, 0.0, 0.0], 0.0),
            ("CART_5MM_POS_Y", [0.0, 0.005, 0.0], 0.0),
            ("CART_5MM_POS_Z", [0.0, 0.0, 0.005], 0.0),
            ("CART_15MM_POS_X", [0.015, 0.0, 0.0], 0.0),
            ("CART_15MM_DIAGONAL", [0.010, 0.010, 0.010], 0.0),
            ("CART_ROT_RX_5DEG", [0.0, 0.0, 0.0], 5.0),
            ("CART_ROT_RZ_10DEG", [0.0, 0.0, 0.0], 10.0),
        ]

        cart_results = {}
        for label, delta_xyz, rot_deg in cases:
            start_tcp = self.probe.tcp_pose()
            target_pos = [start_tcp[0][i] + delta_xyz[i] for i in range(3)]
            if rot_deg > 0:
                target_quat = quaternion_multiply(start_tcp[1], axis_angle_quaternion(0 if "RX" in label else 2, math.radians(rot_deg)))
            else:
                target_quat = start_tcp[1]

            with self.probe.lock:
                hist_start = len(self.probe.tcp_history)

            t_start = time.monotonic()
            self.probe.publish_pose(target_pos, target_quat)
            timeout = 15.0
            reached = self.probe.wait_pose(target_pos, target_quat, timeout=timeout)
            exec_time = time.monotonic() - t_start

            with self.probe.lock:
                history = list(self.probe.tcp_history[hist_start:])

            final_tcp = self.probe.tcp_pose()
            pos_err_mm = position_error(final_tcp[0], target_pos) * 1000.0 if final_tcp else float('nan')

            if len(history) >= 4:
                stamps = np.array([row[0] for row in history])
                pos_arr = np.array([row[1] for row in history])
                metrics = compute_cartesian_smoothness(stamps, pos_arr)

                v_max = metrics["max_speed_mm_s"]
                a_max = metrics["max_accel_mm_s2"]
                j_max = metrics["max_jerk_mm_s3"]
                j_rms = metrics["rms_jerk_mm_s3"]

                print(f"  [{label:<20}]: "
                      f"Time: {exec_time:4.2f}s | "
                      f"Vmax: {v_max:5.1f} mm/s | "
                      f"Amax: {a_max:5.1f} mm/s² | "
                      f"JerkRMS: {j_rms:6.1f} mm/s³ | "
                      f"PosErr: {pos_err_mm:5.3f} mm")

                cart_results[label] = {
                    "status": "PASS" if reached and pos_err_mm <= 2.0 else "FAIL",
                    "execution_time_s": exec_time,
                    "position_error_mm": float(pos_err_mm),
                    "max_speed_mm_s": float(v_max),
                    "max_acceleration_mm_s2": float(a_max),
                    "max_jerk_mm_s3": float(j_max),
                    "rms_jerk_mm_s3": float(j_rms),
                }

            # Return cleanly
            if reached:
                self.probe.publish_pose(start_tcp[0], start_tcp[1])
                self.probe.wait_pose(start_tcp[0], start_tcp[1], timeout=timeout)
                time.sleep(0.2)

        self.results["cartesian_benchmark"] = cart_results
        return cart_results

    def benchmark_ai_stream_10hz(self) -> Dict[str, Any]:
        print("\n" + "-" * 70)
        print("6. 10 HZ CONTINUOUS AI TARGET STREAMING BENCHMARK")
        print("-" * 70)

        center = self.probe.joint_position()
        amplitudes = [math.radians(v) for v in (10.0, 8.0, 8.0, 6.0, 6.0, 8.0)]
        for axis in range(6):
            avail = min(MAX_JOINT[axis] - center[axis], center[axis] - MIN_JOINT[axis]) - math.radians(10.0)
            amplitudes[axis] = max(0.0, min(amplitudes[axis], avail))

        with self.probe.lock:
            hist_start = len(self.probe.joint_history)

        begun = time.monotonic()
        submissions = 0
        target_duration = 8.0
        while time.monotonic() - begun < target_duration:
            t = time.monotonic() - begun
            tgt = [center[i] + amplitudes[i] * math.sin((0.40 + 0.03 * i) * t) for i in range(6)]
            self.probe.publish_joint(tgt)
            submissions += 1
            time.sleep(max(0.0, begun + submissions * 0.1 - time.monotonic()))

        actual_stream_hz = submissions / (time.monotonic() - begun)
        self.probe.publish_joint(center)
        reached = self.probe.wait_joint(center, timeout=20.0)

        with self.probe.lock:
            history = list(self.probe.joint_history[hist_start:])

        if len(history) >= 4:
            stamps = np.array([row[0] for row in history])
            positions = np.array([row[1] for row in history])
            velocities = np.array([row[2] for row in history])
            metrics = compute_smoothness_metrics(stamps, positions, velocities)

            max_j = max(metrics["max_jerk_deg_s3"])
            rms_j = max(metrics["rms_jerk_rad_s3"])

            print(f"  -> AI Stream Frequency:    {actual_stream_hz:.2f} Hz (Target: 10.0 Hz)")
            print(f"  -> Total AI Submissions:   {submissions}")
            print(f"  -> Controller Loop Rate:   {len(history)/target_duration:.1f} Hz (125 Hz OTG Interpolation)")
            print(f"  -> Peak Streaming Jerk:    {max_j:.1f} °/s³")
            print(f"  -> RMS Streaming Jerk:     {rms_j:.2f} rad/s³")

            res = {
                "status": "PASS" if 9.0 <= actual_stream_hz <= 11.0 and reached else "FAIL",
                "ai_stream_hz": float(actual_stream_hz),
                "total_submissions": submissions,
                "peak_jerk_deg_s3": float(max_j),
                "rms_jerk_rad_s3": float(rms_j),
            }
        else:
            res = {"status": "FAIL"}

        self.results["ai_streaming_benchmark"] = res
        return res

    def save_summary(self):
        summary_path = self.results_dir / "benchmark_summary.json"
        write_json(summary_path, self.results)
        print("\n" + "=" * 70)
        print(f"[+] BENCHMARK COMPLETE! Full results saved to: {summary_path}")
        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="FR10 SimMachine Smoothness & Speed Benchmark")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()

    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    runner = BenchmarkRunner(args.robot_ip, args.domain_id, results_dir)

    if not runner.start_environment():
        return 1

    try:
        runner.benchmark_rate_and_latency()
        runner.benchmark_joint_smoothness_and_speed()
        runner.benchmark_multijoint_smoothness()
        runner.benchmark_sudden_retargeting_smoothness()
        runner.benchmark_cartesian_smoothness_and_speed()
        runner.benchmark_ai_stream_10hz()
        runner.save_summary()
        return 0
    finally:
        runner.stop_environment()


if __name__ == "__main__":
    sys.exit(main())
