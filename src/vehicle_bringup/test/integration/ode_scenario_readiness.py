"""Detect controller activation in a fresh Gazebo launch log."""

import time
from pathlib import Path


def controllers_activated(log_text, required):
    """Return true only after every named spawner reports activation."""
    activated = set()
    for line in log_text.splitlines():
        if 'Configured and activated' not in line:
            continue
        for name in required - activated:
            if f'[spawner_{name}]' in line and name in line.split(
                    'Configured and activated', 1)[1]:
                activated.add(name)
    return activated == required


def wait_for_controllers(log_path: Path, required, launch, timeout_s=55.0):
    """Wait on launch evidence, avoiding concurrent test-client service calls."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if log_path.exists() and controllers_activated(
                log_path.read_text(errors='replace'), required):
            return True
        if launch.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def required_controllers_for_case(case):
    """Controller activation contract for diagnostic vehicle modes V0–V8."""
    if case not in {f'V{i}' for i in range(9)}:
        raise ValueError(f'unknown vehicle case: {case}')
    if case == 'V0':
        return set()
    required = {'joint_state_broadcaster'}
    if case in {'V2', 'V4', 'V5', 'V6', 'V7', 'V8'}:
        required.add('steering_controller')
    if case in {'V3', 'V4', 'V5', 'V6', 'V7', 'V8'}:
        required.add('drive_controller')
    return required
