#!/usr/bin/env python3
"""FAIRINO/ROS error counters from the captured launch log."""

from __future__ import annotations

import pathlib
import re
from typing import Dict


def analyze(path: pathlib.Path) -> Dict[str, int]:
    text = path.read_text(errors="replace") if path.exists() else ""
    hardware_error_lines = re.findall(
        r"(?:\[ERROR\][^\n]*\[FairinoHardwareInterface\]|"
        r"Hardware entered ERROR|hardware[^\n]*unconfigured)",
        text, re.I)
    return {
        "servo_j_code14": len(re.findall(r"ServoJ[^\n]*(?:code\s*=\s*14|rc\s*=\s*14)", text, re.I)),
        "joint_read_minus2": len(re.findall(r"GetActualJointPosDegree[^\n]*-2", text)),
        "rpc_errors": len(re.findall(r"RPC[^\n]*(?:error|failed)", text, re.I)),
        "hardware_errors": len(hardware_error_lines),
        "sdk_command_errors": len(re.findall(r"command failed: rc=", text)),
        "segfaults": len(re.findall(r"Segmentation fault", text)),
    }
