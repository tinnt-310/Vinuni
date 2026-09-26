#!/usr/bin/env python3
"""Deterministic joint target generation from actual feedback."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Sequence


MIN_JOINT = (-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543)
MAX_JOINT = (3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543)


@dataclass
class SafeTarget:
    allowed: bool
    joint_index: int
    requested_delta_deg: float
    actual_delta_deg: float
    direction: int
    target: list[float]
    blocked_reason: str = ""

    def json(self):
        return asdict(self)


def generate(actual_q: Sequence[float], joint_index: int,
             requested_delta_deg: float, safety_margin_deg: float = 10.0,
             allow_clamp: bool = False) -> SafeTarget:
    requested = math.radians(abs(requested_delta_deg))
    margin = math.radians(safety_margin_deg)
    positive = MAX_JOINT[joint_index] - margin - actual_q[joint_index]
    negative = actual_q[joint_index] - (MIN_JOINT[joint_index] + margin)
    direction = 1 if positive >= negative else -1
    available = max(positive, negative)
    target = list(actual_q)
    if available + 1e-12 < requested and not allow_clamp:
        return SafeTarget(False, joint_index, requested_delta_deg, 0.0,
                          direction, target, "BLOCKED_BY_JOINT_MARGIN")
    selected = min(requested, max(0.0, available))
    if selected <= 0.0:
        return SafeTarget(False, joint_index, requested_delta_deg, 0.0,
                          direction, target, "BLOCKED_BY_JOINT_MARGIN")
    target[joint_index] += direction * selected
    return SafeTarget(True, joint_index, requested_delta_deg,
                      math.degrees(selected), direction, target)
