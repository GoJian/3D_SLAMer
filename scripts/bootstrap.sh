#!/usr/bin/env bash
# Set up the ROS 2 workspace on the target machine (Jetson or dev box).
# Usage: scripts/bootstrap.sh            # import deps, install rosdeps, build
#        SKIP_BUILD=1 scripts/bootstrap.sh
set -euo pipefail
cd "$(dirname "$0")/.."

: "${ROS_DISTRO:?Source /opt/ros/<distro>/setup.bash first}"

command -v vcs >/dev/null || { echo "Install vcstool: sudo apt install python3-vcstool"; exit 1; }

mkdir -p src/third_party
vcs import --recursive src/third_party < dependencies.repos

rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO"

if [[ -z "${SKIP_BUILD:-}" ]]; then
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi
