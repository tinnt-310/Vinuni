#!/usr/bin/env python3
"""Export recorded FT_Control + MoveL feedback; never connects to a robot."""
import argparse
import csv
import json
import math
from pathlib import Path


def export(run):
    case = run if (run / "result.json").is_file() else run / "ft_movel"
    result = json.loads((case / "result.json").read_text())
    initial = result["initial"]["tcp_mm_deg"]
    goal = result["movel_target_mm_deg"]
    direction = [goal[i] - initial[i] for i in range(3)]
    length = math.sqrt(sum(x * x for x in direction))
    direction = [x / length for x in direction]
    records = [json.loads(line) for line in (case / "samples.jsonl").read_text().splitlines() if line]
    # Preserve duplicates in the original JSONL, but plot only fresh feedback.
    records = [s for s in records if s["fresh"]]
    target = result["configuration"]["force"]["desired_wrench"][2]
    rows = [(s["t_s"], s["ft_reference"][2], target,
             sum((s["tcp_mm_deg"][i] - initial[i]) * direction[i] for i in range(3)))
            for s in records]
    with (case / "force_movel.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "reference_fz_n", "target_fz_n", "stroke_progress_mm"])
        writer.writerows(rows)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    times = [r[0] for r in rows]
    axes[0].plot(times, [r[1] for r in rows], label="Measured reference Fz")
    axes[0].axhline(target, color="tab:red", linestyle="--", label="Target Fz")
    axes[0].set_ylabel("Force (N)")
    axes[0].legend()
    axes[1].plot(times, [r[3] for r in rows], label="Measured stroke")
    axes[1].axhline(length, color="tab:red", linestyle="--", label="Target stroke")
    axes[1].set_ylabel("Tangential progress (mm)")
    axes[1].set_xlabel("Time since worker start (s)")
    axes[1].legend()
    events = [json.loads(line) for line in (case / "events.jsonl").read_text().splitlines() if line]
    for event in events:
        if event["event"] == "call_begin" and event["api"] in ("MoveL", "StopMotion/cleanup"):
            for axis in axes:
                axis.axvline(event["t_s"], color="grey", alpha=0.5)
    for axis in axes:
        axis.grid(alpha=0.3)
    fig.suptitle(f"{result['backend']} | {result['status']} | feedback, not external metrology")
    fig.tight_layout()
    fig.savefig(case / "force_movel.png", dpi=160)
    plt.close(fig)
    print(case / "force_movel.csv")
    print(case / "force_movel.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Run directory, or its ft_movel subdirectory")
    args = parser.parse_args()
    export(args.run)
