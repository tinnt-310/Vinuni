#!/usr/bin/env bash
# Source the canonical RB V3 + FR10 ROS 2 overlay from any workspace location.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced:" >&2
  echo "  source \"$(cd "$(dirname "$0")/.." && pwd -P)/tools/source_simulation.sh\"" >&2
  exit 2
fi

_rb_v3_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
_rb_v3_runtime_link="$_rb_v3_workspace_root/ros2_current"
_rb_v3_marker="$_rb_v3_runtime_link/.workspace-root"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
  return 2
fi

if [[ ! -f "$_rb_v3_runtime_link/install/setup.bash" ]]; then
  echo "The simulation runtime has not been built." >&2
  echo "Run: $_rb_v3_workspace_root/tools/build_simulation.sh" >&2
  return 2
fi

if [[ ! -f "$_rb_v3_marker" ]] || [[ "$(<"$_rb_v3_marker")" != "$_rb_v3_workspace_root" ]]; then
  echo "The workspace was moved after its last build." >&2
  echo "Run: $_rb_v3_workspace_root/tools/build_simulation.sh" >&2
  return 2
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
unset ROS_PACKAGE_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH
unset LD_LIBRARY_PATH

# Snap can inject Qt plugin paths that make the APT RViz/Gazebo binaries fail.
if [[ "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == /snap/* ]]; then
  unset QT_QPA_PLATFORM_PLUGIN_PATH
fi
if [[ "${QT_PLUGIN_PATH:-}" == /snap/* ]]; then
  unset QT_PLUGIN_PATH
fi

_rb_v3_restore_nounset=false
case "$-" in
  *u*)
    _rb_v3_restore_nounset=true
    set +u
    ;;
esac

source /opt/ros/humble/setup.bash
source "$_rb_v3_runtime_link/install/setup.bash"

if [[ "$_rb_v3_restore_nounset" == true ]]; then
  set -u
fi

export RB_V3_WORKSPACE_ROOT="$_rb_v3_workspace_root"
export RB_V3_RUNTIME_ROOT="$_rb_v3_runtime_link"

unset _rb_v3_workspace_root _rb_v3_runtime_link _rb_v3_marker _rb_v3_restore_nounset
