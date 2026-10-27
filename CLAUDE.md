# CLAUDE.md

Guidance for Claude Code working in this repository.

## Source of truth

- `docs/3D_SLAMer_TechnicalPlan.md` is the blueprint. `docs/milestones.md` tracks progress.
- Work in milestone order (M0 -> M9). Do not start Gaussian work (M9) before the mesh product passes validation.
- Record significant choices in `docs/decisions/` and tick checklist items in `docs/milestones.md`.

## Layout

- Repo root is the colcon workspace; ROS 2 packages live in `src/`. Third-party sources go in
  `src/third_party/` via `vcs import` from `dependencies.repos` (git-ignored).
- `offline/` holds post-flight workstation tools. It has `COLCON_IGNORE` and must not share an
  environment with flight software.
- Onboard platform: JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy (decision 0001).
- PX4 SITL lives outside the workspace (`scripts/setup_px4_sitl.sh`, default `~/px4`); PX4 ships a
  `package.xml`, so never put it under `src/`. Flight test: `ros2 run x650_sim x650_sim_flight_test`.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
scripts/bootstrap.sh                        # import deps, rosdep, build
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> && colcon test-result --verbose
scripts/validate_config.py                  # needs jsonschema + pyyaml
scripts/capture_env.sh                      # versions for dependency locks
```

## Conventions (do not violate)

- `T_A_B` maps coordinates from frame B into frame A; `T_W_Ci = T_W_B * T_B_Ci`. State direction and units in every stored transform.
- Frames: `map`, `odom`, `base_link` (FLU), `imu_link`, `cam_i_optical` (x right, y down, z forward). REP-103 inside ROS; convert explicitly at the PX4 boundary (NED/FRD).
- Single `map -> odom -> base_link` chain. Discrete global corrections go in `map -> odom` only.
- Keep acquisition timestamps, clock domain, sequence numbers, exposure metadata. Never use callback arrival time as acquisition time.
- Depth: `sensor_msgs/Image` `32FC1` meters; invalid values explicitly defined. Invalid depth never clears obstacles.
- Raw and rectified images on distinct topics. Never undistort twice.
- Bounded queues everywhere. Keyframe events reference persisted files, not full images.
- Navigation must not depend on the Gaussian mapper.
- Measurements use the reference mesh, not display LODs, Gaussian centers or texture pixels.

## Data

- Never commit recordings, bags, images, models or large binaries. Use `data/` (git-ignored).
- Calibrations are versioned in `calibration/rigs/`; never edit one in place.
- Do not claim performance or accuracy without a report in `reports/` backed by a recording.
