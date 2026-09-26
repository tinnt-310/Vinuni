#!/usr/bin/env python3
"""Numeric metrics with no ROS dependency."""

from __future__ import annotations

import math
import statistics
from typing import Dict, Sequence


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    scaled = fraction * (len(sorted_values) - 1)
    lower = math.floor(scaled)
    upper = math.ceil(scaled)
    blend = scaled - lower
    return (sorted_values[lower] +
            blend * (sorted_values[upper] - sorted_values[lower]))


def rate_metrics(stamps: Sequence[float]) -> Dict[str, float]:
    if len(stamps) < 2:
        return {"sample_count": len(stamps), "mean_hz": 0.0,
                "mean_dt": 0.0, "min_dt": 0.0, "max_dt": 0.0,
                "stddev_dt": 0.0, "p95_dt": 0.0, "p99_dt": 0.0,
                "missed_deadlines": 0}
    deltas = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not deltas:
        return {"sample_count": len(stamps), "mean_hz": 0.0,
                "mean_dt": 0.0, "min_dt": 0.0, "max_dt": 0.0,
                "stddev_dt": 0.0, "p95_dt": 0.0, "p99_dt": 0.0,
                "missed_deadlines": 0}
    mean_dt = statistics.fmean(deltas)
    sorted_deltas = sorted(deltas)
    return {
        "sample_count": len(stamps), "mean_hz": 1.0 / mean_dt,
        "mean_dt": mean_dt, "min_dt": min(deltas), "max_dt": max(deltas),
        "stddev_dt": statistics.pstdev(deltas),
        "p95_dt": _percentile(sorted_deltas, 0.95),
        "p99_dt": _percentile(sorted_deltas, 0.99),
        # /joint_states is nominally 125 Hz in this package. A period above
        # 1.5 x nominal is recorded, not interpreted as a certified deadline.
        "missed_deadlines": sum(delta > 0.012 for delta in deltas),
    }


def max_abs_error(actual: Sequence[float], target: Sequence[float]) -> float:
    return max(abs(a - b) for a, b in zip(actual, target))


def position_error(actual: Sequence[float], target: Sequence[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(actual, target)))


def quaternion_error(first: Sequence[float], second: Sequence[float]) -> float:
    n1 = math.sqrt(sum(value * value for value in first))
    n2 = math.sqrt(sum(value * value for value in second))
    if n1 <= 1e-12 or n2 <= 1e-12:
        return math.inf
    dot = abs(sum(a * b for a, b in zip(first, second)) / (n1 * n2))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))
