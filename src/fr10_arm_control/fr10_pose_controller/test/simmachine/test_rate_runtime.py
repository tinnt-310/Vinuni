#!/usr/bin/env python3
"""Runtime Frequency and Jitter measurement test for FAIRINO FR10.

Measures:
1. High-Level AI publishing rate (target ~10 Hz).
2. Low-Level Control Loop Rate on /joint_states:
   - Sample count
   - Mean frequency (Hz)
   - Mean dt, Min dt, Max dt, Stddev dt
3. Saves rate.json with full numeric timing metrics.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

from common import (
    MAX_JOINT,
    MIN_JOINT,
    ManagedLaunch,
    ProbeContext,
    check_preflight,
    make_results_dir,
    rate_metrics,
    write_json,
)


def run_rate_test(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                  results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running rate tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    outcomes = {}

    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [PHASE 1] Measuring Low-Level /joint_states Frequency (10s window) ===")
        with probe.lock:
            start_index = len(probe.joint_stamps)
        time.sleep(10.0)
        with probe.lock:
            stamps = list(probe.joint_stamps[start_index:])
            status = probe.latest_status

        timing = rate_metrics(stamps)
        rate_ok = (len(stamps) > 100 and 115.0 <= timing["mean_hz"] <= 135.0 and
                   "sdk_errors=0" in status)
        outcomes["LOW_LEVEL_RATE"] = "PASS" if rate_ok else "FAIL"
        outcomes["timing_metrics"] = timing
        print(f"  Sample count: {timing['sample_count']}")
        print(f"  Mean rate:    {timing['mean_hz']:.2f} Hz")
        print(f"  Mean dt:      {timing['mean_dt']*1000:.3f} ms")
        print(f"  Min dt:       {timing['min_dt']*1000:.3f} ms")
        print(f"  Max dt:       {timing['max_dt']*1000:.3f} ms")
        print(f"  Stddev dt:    {timing['stddev_dt']*1000:.3f} ms")
        print(f"  Low-level Rate Status: {outcomes['LOW_LEVEL_RATE']}")

        print("\n=== [PHASE 2] High-Level 10 Hz Joint AI Stream Test (10s duration) ===")
        center = probe.joint_position()
        amplitudes = [math.radians(v) for v in (12, 8, 8, 6, 6, 8)]
        for axis in range(6):
            available = min(MAX_JOINT[axis] - center[axis], center[axis] - MIN_JOINT[axis]) - math.radians(10)
            amplitudes[axis] = max(0.0, min(amplitudes[axis], available))

        begun = time.monotonic()
        submissions = 0
        while time.monotonic() - begun < 10.0:
            t = time.monotonic() - begun
            target = [center[i] + amplitudes[i] * math.sin((0.35 + 0.03 * i) * t) for i in range(6)]
            probe.publish_joint(target)
            submissions += 1
            time.sleep(max(0.0, begun + submissions * 0.1 - time.monotonic()))

        stream_ended = time.monotonic()
        probe.publish_joint(center)
        reached = probe.wait_joint(center, timeout=30.0)
        settled = time.monotonic()
        ai_input_hz = submissions / (stream_ended - begun)
        settle_latency_s = settled - stream_ended
        stream_ok = (reached and 9.0 <= ai_input_hz <= 11.0 and "sdk_errors=0" in probe.status())
        outcomes["JOINT_AI_10HZ"] = "PASS" if stream_ok else "FAIL"
        outcomes["ai_input_hz"] = ai_input_hz
        print(f"  High-level AI Submissions: {submissions} ({ai_input_hz:.2f} Hz)")
        print(f"  Joint AI 10 Hz Status:    {outcomes['JOINT_AI_10HZ']}")

        write_json(results_dir / "rate.json", {
            "low_level_timing": timing,
            "ai_input_hz": ai_input_hz,
            "submissions": submissions,
            "stream_duration_s": stream_ended - begun,
            "settle_latency_s": settle_latency_s,
            "result": "PASS" if (rate_ok and stream_ok) else "FAIL",
        })

        return 0 if (rate_ok and stream_ok) else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Rate Decoupling & Timing Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_rate_test(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
