#!/usr/bin/env python3
"""Export deterministic FR10 kinematic/control feasibility-basin sweeps.

This utility exercises the repository's analytical IK and safety barriers. It
does not model connector contact, compliance, friction, chamfers, or force, so
its output is not a physical insertion capture basin.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
import subprocess
import sys
from datetime import datetime, timezone


WORKSPACE = pathlib.Path(__file__).resolve().parents[4]
DEFAULT_PROBE = WORKSPACE / "build/fr10_pose_controller/fr10_kinematic_probe"
DEFAULT_MODES = ("x", "pitch", "xy", "x_pitch", "y_yaw")


def inclusive_values(lower: float, upper: float, step: float) -> list[float]:
    if step <= 0.0 or upper < lower:
        raise ValueError("range step must be positive and max >= min")
    count = int(math.floor((upper - lower) / step + 1e-9))
    values = [lower + index * step for index in range(count + 1)]
    if values[-1] < upper - 1e-9:
        values.append(upper)
    return values


def samples_for_mode(args, mode: str):
    translation = inclusive_values(
        args.translation_min_mm, args.translation_max_mm,
        args.translation_step_mm)
    angle = inclusive_values(
        args.angle_min_deg, args.angle_max_deg, args.angle_step_deg)
    zero = {"dx_mm": 0.0, "dy_mm": 0.0, "dz_mm": 0.0,
            "droll_deg": 0.0, "dpitch_deg": 0.0, "dyaw_deg": 0.0}

    if mode in ("x", "y", "z"):
        key = {"x": "dx_mm", "y": "dy_mm", "z": "dz_mm"}[mode]
        for value in translation:
            sample = dict(zero)
            sample[key] = value
            yield sample
    elif mode in ("roll", "pitch", "yaw"):
        key = {"roll": "droll_deg", "pitch": "dpitch_deg",
               "yaw": "dyaw_deg"}[mode]
        for value in angle:
            sample = dict(zero)
            sample[key] = value
            yield sample
    elif mode in ("xy", "xz", "yz"):
        keys = {"xy": ("dx_mm", "dy_mm"),
                "xz": ("dx_mm", "dz_mm"),
                "yz": ("dy_mm", "dz_mm")}[mode]
        for first in translation:
            for second in translation:
                sample = dict(zero)
                sample[keys[0]], sample[keys[1]] = first, second
                yield sample
    elif mode in ("roll_pitch", "roll_yaw", "pitch_yaw"):
        keys = {"roll_pitch": ("droll_deg", "dpitch_deg"),
                "roll_yaw": ("droll_deg", "dyaw_deg"),
                "pitch_yaw": ("dpitch_deg", "dyaw_deg")}[mode]
        for first in angle:
            for second in angle:
                sample = dict(zero)
                sample[keys[0]], sample[keys[1]] = first, second
                yield sample
    elif mode in ("x_pitch", "y_yaw"):
        keys = {"x_pitch": ("dx_mm", "dpitch_deg"),
                "y_yaw": ("dy_mm", "dyaw_deg")}[mode]
        for linear in translation:
            for angular in angle:
                sample = dict(zero)
                sample[keys[0]], sample[keys[1]] = linear, angular
                yield sample
    elif mode == "xyz_coarse":
        for dx in translation:
            for dy in translation:
                for dz in translation:
                    sample = dict(zero)
                    sample["dx_mm"], sample["dy_mm"], sample["dz_mm"] = dx, dy, dz
                    yield sample
    else:
        raise ValueError(f"unsupported sweep mode: {mode}")


def probe_sample(probe: pathlib.Path, sample: dict) -> dict:
    command = [
        str(probe), "--sample",
        str(sample["dx_mm"] / 1000.0),
        str(sample["dy_mm"] / 1000.0),
        str(sample["dz_mm"] / 1000.0),
        str(math.radians(sample["droll_deg"])),
        str(math.radians(sample["dpitch_deg"])),
        str(math.radians(sample["dyaw_deg"])),
    ]
    completed = subprocess.run(
        command, text=True, capture_output=True, timeout=5.0, check=False)
    if completed.returncode != 0:
        return {
            "accepted": False, "failure_reason": "PROBE_ERROR",
            "ik_success": False, "solver_message": completed.stderr.strip(),
            "execution_time_us": 0.0,
        }
    return json.loads(completed.stdout)


def write_svg(path: pathlib.Path, mode: str, rows: list[dict]) -> bool:
    axes = {
        "xy": ("dx_mm", "dy_mm"),
        "x_pitch": ("dx_mm", "dpitch_deg"),
        "y_yaw": ("dy_mm", "dyaw_deg"),
        "xz": ("dx_mm", "dz_mm"),
        "yz": ("dy_mm", "dz_mm"),
        "roll_pitch": ("droll_deg", "dpitch_deg"),
        "roll_yaw": ("droll_deg", "dyaw_deg"),
        "pitch_yaw": ("dpitch_deg", "dyaw_deg"),
    }.get(mode)
    if axes is None:
        return False
    xs = sorted({row[axes[0]] for row in rows})
    ys = sorted({row[axes[1]] for row in rows})
    cell = 18
    left, top = 70, 65
    width = left + cell * len(xs) + 30
    height = top + cell * len(ys) + 70
    lookup = {(row[axes[0]], row[axes[1]]): row for row in rows}
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:sans-serif;font-size:11px}</style>',
        f'<text x="10" y="18" font-size="14">{mode}: offline analytical kinematic/control feasibility</text>',
        '<text x="10" y="36" fill="#a00">NOT a physical connector capture basin</text>',
    ]
    for yi, yvalue in enumerate(reversed(ys)):
        for xi, xvalue in enumerate(xs):
            row = lookup[(xvalue, yvalue)]
            color = "#2ca25f" if row["accepted"] else "#de2d26"
            svg.append(
                f'<rect x="{left + xi * cell}" y="{top + yi * cell}" '
                f'width="{cell - 1}" height="{cell - 1}" fill="{color}">' 
                f'<title>{axes[0]}={xvalue}, {axes[1]}={yvalue}, '
                f'{row["failure_reason"]}</title></rect>')
    svg.extend([
        f'<text x="{left}" y="{height - 38}">{axes[0]}: {xs[0]} .. {xs[-1]}</text>',
        f'<text x="10" y="{top + 10}">{axes[1]}: {ys[-1]}</text>',
        f'<text x="10" y="{top + cell * len(ys)}">{ys[0]}</text>',
        '<rect x="10" y="{}" width="12" height="12" fill="#2ca25f"/>'.format(height - 22),
        '<text x="28" y="{}">IK/safety accepted</text>'.format(height - 12),
        '<rect x="150" y="{}" width="12" height="12" fill="#de2d26"/>'.format(height - 22),
        '<text x="168" y="{}">rejected</text>'.format(height - 12),
        '</svg>',
    ])
    path.write_text("\n".join(svg))
    return True


def run_mode(args, mode: str, output_dir: pathlib.Path) -> dict:
    rows = []
    timestamp = datetime.now(timezone.utc).isoformat()
    for index, injected in enumerate(samples_for_mode(args, mode), start=1):
        measured = probe_sample(args.probe, injected)
        rows.append({
            "test_id": f"BASIN_{mode.upper()}_{index:05d}",
            "timestamp_utc": timestamp,
            "environment": "OfflineAnalyticalModel",
            **injected,
            **measured,
            "max_perpendicular_error_m": "",
            "fsm_final_state": "UNAVAILABLE",
        })
    csv_path = output_dir / f"basin_{mode}.csv"
    fields = sorted({key for row in rows for key in row})
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    accepted = sum(1 for row in rows if row["accepted"])
    latencies = [float(row["execution_time_us"]) for row in rows]
    svg_path = output_dir / f"basin_{mode}.svg"
    plotted = write_svg(svg_path, mode, rows)
    return {
        "mode": mode,
        "samples": len(rows),
        "accepted": accepted,
        "rejected": len(rows) - accepted,
        "acceptance_ratio": accepted / len(rows) if rows else 0.0,
        "mean_ik_time_us": statistics.fmean(latencies) if latencies else 0.0,
        "max_ik_time_us": max(latencies, default=0.0),
        "csv": str(csv_path),
        "plot": str(svg_path) if plotted else None,
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", action="append", choices=(
        "x", "y", "z", "roll", "pitch", "yaw", "xy", "xz", "yz",
        "roll_pitch", "roll_yaw", "pitch_yaw", "x_pitch", "y_yaw",
        "xyz_coarse"))
    parser.add_argument("--translation-min-mm", type=float, default=-10.0)
    parser.add_argument("--translation-max-mm", type=float, default=10.0)
    parser.add_argument("--translation-step-mm", type=float, default=2.0)
    parser.add_argument("--angle-min-deg", type=float, default=-5.0)
    parser.add_argument("--angle-max-deg", type=float, default=5.0)
    parser.add_argument("--angle-step-deg", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=20260829)
    parser.add_argument("--probe", type=pathlib.Path, default=DEFAULT_PROBE)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.probe.is_file():
        raise SystemExit(f"probe executable not found: {args.probe}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    modes = tuple(args.mode or DEFAULT_MODES)
    results = [run_mode(args, mode, args.output_dir) for mode in modes]
    metadata = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "classification": "kinematic/control feasibility basin",
        "environment": "OfflineAnalyticalModel",
        "physical_capture_basin": False,
        "seed": args.seed,
        "probe": str(args.probe),
        "ranges": {
            "translation_mm": [args.translation_min_mm,
                               args.translation_max_mm,
                               args.translation_step_mm],
            "angle_deg": [args.angle_min_deg, args.angle_max_deg,
                          args.angle_step_deg],
        },
        "results": results,
    }
    summary_path = args.output_dir / "capture_basin_summary.json"
    summary_path.write_text(json.dumps(metadata, indent=2, sort_keys=True))
    print(summary_path.read_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
