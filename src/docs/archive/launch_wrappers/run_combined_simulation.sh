#!/usr/bin/env bash
# Launch the RB V3 with the FR10 seated in its CAD mounting pocket.
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$workspace_root/tools/source_simulation.sh"

exec ros2 launch vehicle_bringup simulation.launch.py \
  enable_vehicle:=true \
  enable_arm:=true \
  arm_controller_profile:=pose \
  rviz:=true \
  "$@"
