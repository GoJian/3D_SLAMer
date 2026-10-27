#!/usr/bin/env bash
# Install PX4 SITL (Gazebo Harmonic) for the M3 flight tests, outside the colcon workspace.
#   scripts/setup_px4_sitl.sh                 # PX4 into ~/px4/PX4-Autopilot
#   PX4_DIR=/opt/px4 scripts/setup_px4_sitl.sh
# Pins: config/dependencies.jetson.lock.yaml (px4_firmware, gazebo). Decision 0004.
# PX4 ships a package.xml, so it must NOT live under src/. Its Python build deps go in a venv
# (~/px4/venv) instead of PX4's default `pip --break-system-packages`.
set -euo pipefail
PX4_VERSION=v1.17.0
PX4_DIR=${PX4_DIR:-$HOME/px4/PX4-Autopilot}
VENV=${PX4_VENV:-$(dirname "$PX4_DIR")/venv}

if ! command -v gz >/dev/null || ! gz sim --versions 2>/dev/null | grep -q '^8\.'; then
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update
fi
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  gz-harmonic cppzmq-dev libunwind-dev ros-"${ROS_DISTRO:-jazzy}"-ros-gz-bridge \
  cmake ninja-build g++ gcc git make python3-venv python3-dev rsync unzip zip \
  libeigen3-dev libopencv-dev libxml2-utils pkg-config protobuf-compiler \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev

if [[ ! -d "$PX4_DIR/.git" ]]; then
  mkdir -p "$(dirname "$PX4_DIR")"
  git clone --branch "$PX4_VERSION" --depth 1 --recursive --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi
actual=$(git -C "$PX4_DIR" describe --tags)
[[ "$actual" == "$PX4_VERSION" ]] || { echo "PX4 at $actual, expected $PX4_VERSION"; exit 1; }

[[ -d "$VENV" ]] || python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -q -r "$PX4_DIR/Tools/setup/requirements.txt"
(cd "$PX4_DIR" && source "$VENV/bin/activate" && make px4_sitl)

echo "PX4 $PX4_VERSION SITL built in $PX4_DIR"
echo "Next: colcon build (needs px4_msgs + microxrcedds_agent from dependencies.repos), then"
echo "  ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_faults"
