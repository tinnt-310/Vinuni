#!/usr/bin/env python3
"""Generate numerical summaries and plots only from recorded campaign data."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import re
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _percentile(values, fraction):
    return float(np.percentile(np.asarray(values), fraction * 100.0)) if values else 0.0


def _joint_csv(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    stamps = np.asarray([float(row["monotonic_s"]) for row in rows])
    positions = np.asarray([
        [float(row[f"q{axis}_rad"]) for axis in range(1, 7)]
        for row in rows])
    if len(stamps):
        stamps = stamps - stamps[0]
    return stamps, positions


def _derivatives(stamps, positions):
    if len(stamps) < 4:
        empty = np.zeros_like(positions)
        return empty, empty, empty
    velocity = np.gradient(positions, stamps, axis=0, edge_order=1)
    acceleration = np.gradient(velocity, stamps, axis=0, edge_order=1)
    jerk = np.gradient(acceleration, stamps, axis=0, edge_order=1)
    return velocity, acceleration, jerk


def _trajectory_metrics(path):
    stamps, positions = _joint_csv(path)
    velocity, acceleration, jerk = _derivatives(stamps, positions)
    periods = np.diff(stamps).tolist()
    return {
        "test_id": path.stem,
        "source_csv": str(path),
        "sample_count": len(stamps),
        "duration_s": float(stamps[-1]) if len(stamps) else 0.0,
        "mean_rate_hz": 1.0 / statistics.fmean(periods) if periods else 0.0,
        "period_p95_ms": _percentile(periods, 0.95) * 1000.0,
        "period_p99_ms": _percentile(periods, 0.99) * 1000.0,
        "max_joint_displacement_deg": float(
            np.max(np.abs(positions - positions[0])) * 180.0 / math.pi)
        if len(positions) else 0.0,
        "peak_observed_velocity_rad_s": float(np.max(np.abs(velocity)))
        if len(velocity) else 0.0,
        "peak_observed_acceleration_rad_s2": float(np.max(np.abs(acceleration)))
        if len(acceleration) else 0.0,
        "peak_observed_raw_jerk_rad_s3": float(np.max(np.abs(jerk)))
        if len(jerk) else 0.0,
        "derivative_note": (
            "Numerical derivatives of SimMachine position feedback; jerk is raw, "
            "not a certified robot smoothness measurement"
        ),
    }


def _save_figure(path, title=None):
    if title:
        plt.suptitle(title)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def _plot_joint_target_actual(campaign, output):
    summary = json.loads((campaign / "joint_directions/joint_direction_summary.json").read_text())
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=False)
    for axis in range(6):
        item = summary[f"JNT-{axis * 2 + 1:02d}"]
        source = campaign / "joint_directions" / f"jnt-{axis * 2 + 1:02d}_j{axis + 1}_pos.csv"
        stamps, positions = _joint_csv(source)
        target = item["target_rad"][axis]
        panel = axes.flat[axis]
        panel.plot(stamps, positions[:, axis], label="actual")
        panel.axhline(target, color="tab:orange", linestyle="--", label="target")
        panel.set_title(f"J{axis + 1} +5 deg")
        panel.set_ylabel("rad")
        panel.grid(True, alpha=0.3)
    axes.flat[-1].set_xlabel("time (s)")
    axes.flat[0].legend()
    _save_figure(output / "01_joint_target_vs_actual.png", "Small Joint target vs actual")


def _plot_joint_error(campaign, output):
    summary = json.loads((campaign / "joint_directions/joint_direction_summary.json").read_text())
    plt.figure(figsize=(10, 6))
    for axis in range(6):
        item = summary[f"JNT-{axis * 2 + 1:02d}"]
        source = campaign / "joint_directions" / f"jnt-{axis * 2 + 1:02d}_j{axis + 1}_pos.csv"
        stamps, positions = _joint_csv(source)
        error_deg = np.degrees(item["target_rad"][axis] - positions[:, axis])
        plt.plot(stamps, error_deg, label=f"J{axis + 1}")
    plt.xlabel("time (s)")
    plt.ylabel("target - actual (deg)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    _save_figure(output / "02_joint_tracking_error.png", "Joint tracking error")


def _plot_derivatives(campaign, output):
    source = campaign / "joint_multi_small/jnt-22.csv"
    stamps, positions = _joint_csv(source)
    velocity, acceleration, jerk = _derivatives(stamps, positions)
    for index, (values, name, unit) in enumerate((
            (velocity, "joint_velocity", "rad/s"),
            (acceleration, "joint_acceleration", "rad/s^2"),
            (jerk, "joint_raw_jerk", "rad/s^3")), start=3):
        plt.figure(figsize=(11, 6))
        for axis in range(6):
            plt.plot(stamps, values[:, axis], label=f"J{axis + 1}")
        plt.xlabel("time (s)")
        plt.ylabel(unit)
        plt.grid(True, alpha=0.3)
        plt.legend(ncol=3)
        suffix = " (raw numerical derivative)" if "jerk" in name else ""
        _save_figure(output / f"{index:02d}_{name}.png", name.replace("_", " ") + suffix)


def _plot_tcp(campaign, output):
    source = campaign / "cartesian_basic/cartesian_small.csv"
    with source.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    stamps = np.asarray([float(row["monotonic_s"]) for row in rows])
    stamps -= stamps[0]
    xyz = np.asarray([[float(row[key]) for key in ("x", "y", "z")] for row in rows])
    target = xyz[0].copy()
    target[0] += 0.005
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for axis, name in enumerate("XYZ"):
        axes[axis].plot(stamps, xyz[:, axis], label="actual")
        axes[axis].axhline(target[axis], color="tab:orange", linestyle="--", label="target")
        axes[axis].set_ylabel(f"{name} (m)")
        axes[axis].grid(True, alpha=0.3)
    axes[0].legend()
    axes[-1].set_xlabel("time (s)")
    _save_figure(output / "06_tcp_target_vs_actual_xyz.png", "CART +X 5 mm TCP")

    error_mm = np.linalg.norm(xyz - target, axis=1) * 1000.0
    direction = target - xyz[0]
    relative = xyz - xyz[0]
    cross = np.cross(relative, direction)
    perpendicular_mm = np.linalg.norm(cross, axis=1) / np.linalg.norm(direction) * 1000.0
    plt.figure(figsize=(10, 5))
    plt.plot(stamps, error_mm, label="TCP target error")
    plt.plot(stamps, perpendicular_mm, label="perpendicular path error")
    plt.xlabel("time (s)")
    plt.ylabel("mm")
    plt.grid(True, alpha=0.3)
    plt.legend()
    _save_figure(output / "07_08_tcp_error_and_perpendicular.png", "TCP errors")


def _plot_timing(campaign, output):
    source = campaign / "joint_multi_small/jnt-22.csv"
    with source.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    stamps = [float(row["monotonic_s"]) for row in rows]
    periods_ms = np.diff(stamps) * 1000.0
    plt.figure(figsize=(9, 5))
    plt.hist(periods_ms, bins=50)
    plt.axvline(np.percentile(periods_ms, 95), color="tab:orange", label="P95")
    plt.axvline(np.percentile(periods_ms, 99), color="tab:red", label="P99")
    plt.xlabel("joint-state callback period (ms)")
    plt.ylabel("count")
    plt.legend()
    _save_figure(output / "10_control_cycle_histogram.png", "Active controller feedback timing")


def _plot_retarget_latency(campaign, output):
    data = json.loads((campaign / "joint_retarget_small/joint_retarget_matrix.json").read_text())
    labels, values = [], []
    for test_id, item in data.items():
        if "retarget_accept_latency_ms" in item:
            labels.append(test_id)
            values.append(item["retarget_accept_latency_ms"])
    plt.figure(figsize=(9, 5))
    plt.bar(labels, values)
    plt.ylabel("accept latency (ms)")
    plt.xticks(rotation=45)
    _save_figure(output / "11_retarget_latency.png", "Joint retarget acceptance latency")


def _plot_ik_latency(campaign, output):
    values = []
    for path in (campaign / "offline_basin").glob("basin_*.csv"):
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                value = row.get("execution_time_us")
                if value:
                    values.append(float(value))
    plt.figure(figsize=(9, 5))
    plt.hist(values, bins=60)
    plt.xlabel("analytical IK time (us)")
    plt.ylabel("count")
    _save_figure(output / "12_ik_latency_histogram.png", "Offline analytical IK latency")


def _plot_singularity(campaign, output):
    text = (campaign / "joint_medium_directions/runtime.log").read_text(errors="replace")
    conditions = [float(value) for value in re.findall(r"condition=([0-9.]+)", text)]
    plt.figure(figsize=(9, 5))
    plt.plot(range(1, len(conditions) + 1), conditions, marker="o")
    plt.axhline(30.0, color="tab:red", linestyle="--", label="configured maximum")
    plt.xlabel("validated goal index")
    plt.ylabel("Jacobian condition number")
    plt.grid(True, alpha=0.3)
    plt.legend()
    _save_figure(output / "13_singularity_condition.png", "Accepted Joint goals")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    csv_paths = sorted(args.campaign_dir.glob("**/*.csv"))
    joint_paths = []
    for path in csv_paths:
        try:
            with path.open() as stream:
                header = stream.readline()
            if "monotonic_s" in header and "q1_rad" in header:
                joint_paths.append(path)
        except OSError:
            continue
    metrics = [_trajectory_metrics(path) for path in joint_paths]
    (args.output_dir / "trajectory_metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    if metrics:
        with (args.output_dir / "trajectory_metrics.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
            writer.writeheader()
            writer.writerows(metrics)

    _plot_joint_target_actual(args.campaign_dir, args.output_dir)
    _plot_joint_error(args.campaign_dir, args.output_dir)
    _plot_derivatives(args.campaign_dir, args.output_dir)
    _plot_tcp(args.campaign_dir, args.output_dir)
    _plot_timing(args.campaign_dir, args.output_dir)
    _plot_retarget_latency(args.campaign_dir, args.output_dir)
    _plot_ik_latency(args.campaign_dir, args.output_dir)
    _plot_singularity(args.campaign_dir, args.output_dir)
    print(json.dumps({
        "joint_trajectories": len(metrics),
        "plots": sorted(path.name for path in args.output_dir.glob("*.png")),
    }, indent=2))


if __name__ == "__main__":
    main()
