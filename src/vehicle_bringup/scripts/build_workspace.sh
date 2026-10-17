#!/usr/bin/env bash
# Build and validate a fresh runtime; never overwrite an active library.
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
if [[ "${1:-}" == "--clean" ]]; then
  # Compatibility: every release is already a clean build. Older releases may
  # still be loaded by Gazebo and must not be deleted by this operation.
  shift
fi

if [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--clean]" >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
  echo "Install dependencies with: $workspace_root/src/vehicle_bringup/scripts/install_workspace_dependencies.sh" >&2
  exit 2
fi

if [[ -e "$workspace_root/ros2_current" && ! -L "$workspace_root/ros2_current" ]]; then
  echo "ros2_current must be a runtime symlink; refusing to replace a directory." >&2
  exit 2
fi
mkdir -p "$workspace_root/.colcon/releases"
runtime_root="$(mktemp -d "$workspace_root/.colcon/releases/rb_v3_humble_$(date -u +%Y%m%dT%H%M%SZ).XXXXXX")"
mkdir -p "$runtime_root/build" "$runtime_root/install" "$runtime_root/log"

# Do not let another ROS workspace or a Snap desktop session contaminate this
# build. The resulting overlay has only /opt/ros/humble as its underlay.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
unset ROS_PACKAGE_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH
unset LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=4

colcon --log-base "$runtime_root/log" build \
  --base-paths "$workspace_root/src" \
  --build-base "$runtime_root/build" \
  --install-base "$runtime_root/install" \
  --parallel-workers 4 \
  --packages-up-to vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    '-Ufr10*_DIR' '-Ufairino*_DIR' '-Uvehicle*_DIR' \
    '-Uswerve*_DIR' '-Ucharger*_DIR' '-Uccs2*_DIR'

# Install copies keep binaries, profiles and launch files at one revision.
set +u
source "$runtime_root/install/setup.bash"
set -u
export ROS_LOG_DIR="$runtime_root/log/ros"
mkdir -p "$ROS_LOG_DIR"
colcon --log-base "$runtime_root/log" test --base-paths "$workspace_root/src" --build-base "$runtime_root/build" \
  --install-base "$runtime_root/install" --return-code-on-test-failure \
  --parallel-workers 4 --packages-up-to vehicle_bringup
colcon test-result --test-result-base "$runtime_root/build"
printf '%s\n' "$workspace_root" > "$runtime_root/.workspace-root"

# A failed build/test leaves the previous entry point untouched. Existing
# processes retain their original prefix and loaded files after publication.
runtime_link="$workspace_root/.ros2_current-$$"
trap 'rm -f "$runtime_link"' EXIT
ln -s "${runtime_root#"$workspace_root/"}" "$runtime_link"
mv -Tf "$runtime_link" "$workspace_root/ros2_current"

echo
echo "Runtime ready: $workspace_root/ros2_current"
echo "Next: source \"$workspace_root/install/setup.bash\""
