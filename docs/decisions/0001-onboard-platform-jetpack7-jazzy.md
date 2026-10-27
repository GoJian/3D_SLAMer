# 0001: Onboard platform is JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy

- Status: accepted
- Date: 2026-10-27
- Milestone: M0

## Context

The repository assumed JetPack 6 / Ubuntu 22.04 / ROS 2 Humble. The companion computer as
delivered is a Seeed reComputer Robotics J401 (GMSL) carrier with a Jetson Orin NX 16 GB, flashed
with Seeed image `mfi_recomputer-robo-orin-nx-16g-j401-gmsl-7.2.0-39.2.0-2026-08-19`:
JetPack 7.2 (L4T R39.2.0), Ubuntu 24.04, CUDA 13.2. Humble has no Ubuntu 24.04 binaries.

## Decision

- Target ROS 2 **Jazzy** (Ubuntu 24.04 binaries from packages.ros.org) on the current JetPack 7.2
  image. Do not reflash to JetPack 6 just to keep Humble.
- CI builds in `ros:jazzy` on `ubuntu-24.04`.
- `x650_common` (header-only, Eigen) holds shared rigid-body and time utilities so camera, adapter
  and bridge use one implementation.

## Consequences

- Third-party sources must support Jazzy. OKVIS2-X, px4_msgs, nvblox / Isaac ROS and the
  zed-ros2-wrapper must each be verified against Jazzy + JetPack 7.2 when imported.
- **Open risk: ZED X support on this carrier.** Seeed documents ZED X GMSL on JetPack 7.2 only for
  the J50 Mini (driver `stereolabs-zedx-j501-mini_1.4.3.7-L4T39.2.0`, or a BSP dated 2026-09-24 or
  later); the J40-series guide is "coming soon". This image's BSP predates that cutoff. A Stereolabs
  forum thread reports a driver install error on JetPack 7.2 with ZED Link. Before buying or
  wiring the camera, confirm with Seeed/Stereolabs which driver package and BSP support the
  J401 GMSL carrier, and pin the matching "ZED SDK for JetPack 7.2 (L4T 39.2)" release.
- The ZED Link capture card listed in `config/hardware/x650_rev0.yaml` is likely unnecessary: the
  carrier has native GMSL2 ports. Confirm during procurement.

## Evidence

- `scripts/capture_env.sh` output, 2026-10-27, host `jatom1` (copied into
  `config/dependencies.jetson.lock.yaml`).
- Seeed wiki: https://wiki.seeedstudio.com/zed_x_gmsl_cameras_on_recomputer_robotics/
- Stereolabs forum: https://community.stereolabs.com/t/zed-driver-installation-error-with-jetpack-7-2/11645
