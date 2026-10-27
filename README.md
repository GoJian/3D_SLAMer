# 3D_SLAMer

X650-based aerial development platform: continuous stereo-inertial odometry for PX4, synchronized
RGB/IMU recording, and a post-flight **metric textured mesh** you can navigate and measure.
Gaussian splats are an optional appearance product.

The blueprint is [`docs/3D_SLAMer_TechnicalPlan.md`](docs/3D_SLAMer_TechnicalPlan.md).
Progress is tracked in [`docs/milestones.md`](docs/milestones.md).

**Status:** scaffold only. Nothing has been built or tested yet. Starting point: M0 (dependency lock).

## Layout

```text
.
├── src/                         ROS 2 packages (repo root is the colcon workspace)
│   ├── x650_msgs                custom messages (keyframe events, pose updates, mapping health)
│   ├── x650_description         URDF + static TF from calibration
│   ├── x650_camera              ZED X acquisition wrapper (M1)
│   ├── x650_recording           rosbag2/MCAP sessions + manifests (M1)
│   ├── x650_estimator_adapter   OKVIS2-X / cuVSLAM normalization, map->odom->base_link (M2)
│   ├── x650_px4_bridge          frame/time conversion to PX4 external vision (M3)
│   ├── x650_depth               confidence-filtered stereo depth (M4)
│   ├── x650_geometry            local TSDF/occupancy/ESDF (M4)
│   ├── x650_keyframes           persisted keyframe bundles (M5/M9)
│   ├── x650_evaluation          metrics, timing audits, fault injection
│   ├── x650_bringup             launch + parameter composition
│   └── third_party/             vcs-imported deps (git-ignored)
├── offline/                     post-flight workstation tools, separate env (COLCON_IGNORE)
│   ├── x650_reconstruction      COLMAP + OpenMVS orchestration (M5)
│   ├── x650_model_export        GLB / reference mesh / quality package (M5-M6)
│   ├── x650_model_viewer        orbit/fly/walk + measurement viewer (M6)
│   └── x650_gaussian            optional Gaussian mapping (M9)
├── config/                      JSON schemas + hardware, clock policy, dependency locks
├── calibration/                 versioned rig calibrations
├── docker/                      jetson + workstation images (unverified skeletons)
├── scripts/                     bootstrap, environment capture, config validation
├── docs/                        technical plan, milestones, decisions
├── reports/                     small machine-readable evaluation outputs
├── tests/fixtures/              index of frozen regression recordings
├── data/                        recordings and outputs (git-ignored)
└── dependencies.repos           third-party ROS sources (pin during M0)
```

## Getting started on the edge machine

Assumes Ubuntu 22.04 / JetPack 6 with ROS 2 Humble. Confirm and pin during M0.

```bash
git clone git@github.com:GoJian/3D_SLAMer.git
cd 3D_SLAMer
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
scripts/bootstrap.sh            # vcs import + rosdep + colcon build
source install/setup.bash

scripts/capture_env.sh          # copy output into config/dependencies.jetson.lock.yaml
pip install jsonschema pyyaml && scripts/validate_config.py
```

The ZED SDK and ZED Link capture driver are installed manually (not via rosdep); see
`config/dependencies.jetson.lock.yaml`.

Offline tools use their own environment, for example:

```bash
python3 -m venv .venv-offline && . .venv-offline/bin/activate
pip install -e offline/x650_reconstruction -e offline/x650_model_export
```

## Ground rules

- Navigation must keep working when Gaussian mapping is disabled, stalled or restarted.
- One authoritative pose producer per output stream. Loop-closure corrections go into `map -> odom`.
- `T_A_B` maps coordinates from B into A. SI units. Depth is `32FC1` meters.
- Keep acquisition timestamps; never substitute callback arrival time.
- Measurements use validated reference mesh geometry, never Gaussian centers or texture pixels.

## License

Not yet chosen. Dependency licenses (notably Photo-SLAM / Gaussian-Splatting) must be audited first.
