#!/usr/bin/env bash
# Standard Ubuntu 22.04 / ROS 2 Humble dependencies for RB V3 + FR10 simulation.
set -euo pipefail

if [[ "$(. /etc/os-release; printf '%s' "$VERSION_CODENAME")" != "jammy" ]]; then
  echo "This installer supports Ubuntu 22.04 (jammy)." >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed under /opt/ros/humble." >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  python3-pip python3-yaml python3-numpy python3-pytest \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libeigen3-dev libopencv-dev \
  ros-humble-ament-cmake-gmock \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-controller-interface \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers ros-humble-realtime-tools \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-ruckig \
  ros-humble-realsense2-description \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install \
  --from-paths "$workspace_root/src" \
  --ignore-src \
  --rosdistro humble \
  --skip-keys="ament_add_gmock opencv2 warehouse_ros_mongo" \
  -r -y
