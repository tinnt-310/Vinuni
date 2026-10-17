"""Build the separate-process Gazebo pose observer used by regression runs."""

import shlex
import subprocess
from pathlib import Path


def observer_binary():
    source = Path(__file__).with_name('ode_transport_observer.cpp')
    frames = source.with_name('ode_pose_frames.hpp')
    binary = Path('/tmp/fr10_ode_transport_observer')
    if binary.is_file() and binary.stat().st_mtime >= max(source.stat().st_mtime, frames.stat().st_mtime):
        return binary
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    subprocess.run(['g++', '-std=c++17', str(source), '-o', str(binary), *flags],
                   check=True)
    return binary
