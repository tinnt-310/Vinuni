#!/usr/bin/env bash
# Launch the standalone FR10 with the joint/Pose controller enabled.
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$workspace_root/tools/source_simulation.sh"

exec ros2 launch vehicle_bringup simulation.launch.py \
  enable_vehicle:=false \
  enable_arm:=true \
  arm_controller_profile:=pose \
  rviz:=true \
  "$@"
