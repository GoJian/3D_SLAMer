#!/usr/bin/env bash
# M0 helper: print versions of the installed stack so they can be copied into
# config/dependencies.*.lock.yaml. Read-only.
set -u
echo "# captured $(date -u +%Y-%m-%dT%H:%M:%SZ) on $(hostname)"
echo "kernel: $(uname -r)"
[[ -f /etc/nv_tegra_release ]] && echo "l4t: $(head -1 /etc/nv_tegra_release)"
dpkg-query -W -f='jetpack: ${Version}\n' nvidia-jetpack 2>/dev/null || true
[[ -f /etc/os-release ]] && . /etc/os-release && echo "os: $PRETTY_NAME"
echo "ros_distro: ${ROS_DISTRO:-not sourced}"
command -v nvcc >/dev/null && nvcc --version | tail -1
[[ -f /usr/local/zed/include/sl/Camera.hpp ]] && \
  grep -E 'ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp | head -3
command -v colmap >/dev/null && colmap -h 2>&1 | head -1
if [[ -d src/third_party ]]; then
  echo "third_party commits:"
  for d in src/third_party/*/; do
    [[ -d "$d/.git" ]] && echo "  $(basename "$d"): $(git -C "$d" rev-parse HEAD)"
  done
fi
