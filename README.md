# 3D_SLAMer

X650-based aerial development platform: continuous stereo-inertial odometry for PX4, synchronized
RGB/IMU recording, and a post-flight **metric textured mesh** you can navigate and measure.
Gaussian splats are an optional appearance product.

The blueprint is [`docs/3D_SLAMer_TechnicalPlan.md`](docs/3D_SLAMer_TechnicalPlan.md).
Progress is tracked in [`docs/milestones.md`](docs/milestones.md).

**Status:** M0 in progress. Onboard platform pinned (JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy,
[decision 0001](docs/decisions/0001-onboard-platform-jetpack7-jazzy.md)). Hardware-independent
pieces are implemented and tested on the Jetson with **synthetic** data: frame/time conversion,
estimator adapter, calibration loader + static TF, session recorder + manifest, timestamp audit,
trajectory metrics and replay evaluation. The PX4 external-vision bridge flies PX4 v1.17.0 SITL
(Gazebo Harmonic) on vision only, with fault injection
([decision 0004](docs/decisions/0004-px4-transport-uxrce-dds.md),
[report](reports/2026-10-27_m3_sitl.md); fake VIO, not an estimator result). A USB webcam
capture node exists but has not run on a device yet. No real camera, estimator or flight
controller has been connected.

## Layout

```text
.
├── src/                         ROS 2 packages (repo root is the colcon workspace)
│   ├── x650_common              header-only rigid-body + time-mapping utilities (no ROS runtime)
│   ├── x650_msgs                custom messages (frame metadata, keyframe events, pose updates, mapping health)
│   ├── x650_description         URDF + static TF from calibration
│   ├── x650_camera              USB webcam (V4L2) capture now; ZED X wrapper later (M1)
│   ├── x650_recording           rosbag2/MCAP sessions + manifests (M1)
│   ├── x650_estimator_adapter   OKVIS2-X / cuVSLAM normalization, map->odom->base_link (M2)
│   ├── x650_px4_bridge          frame/time conversion to PX4 external vision (M3)
│   ├── x650_depth               confidence-filtered stereo depth (M4)
│   ├── x650_geometry            local TSDF/occupancy/ESDF (M4)
│   ├── x650_keyframes           persisted keyframe bundles (M5/M9)
│   ├── x650_evaluation          metrics, timing audits, fault injection
│   ├── x650_sim                 PX4 SITL harness: fake VIO, offboard mission, flight test (M3)
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

JetPack 7.2 / Ubuntu 24.04 with ROS 2 Jazzy (see `config/dependencies.jetson.lock.yaml`).

```bash
git clone git@github.com:GoJian/3D_SLAMer.git
cd 3D_SLAMer
sudo apt install ros-jazzy-ros-base ros-dev-tools ros-jazzy-rosbag2-storage-mcap \
    python3-jsonschema python3-yaml libeigen3-dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
scripts/validate_config.py

scripts/bootstrap.sh            # later: vcs import third-party (OKVIS2-X, ...) + rosdep + build
scripts/capture_env.sh          # copy output into config/dependencies.jetson.lock.yaml
```

Tools (synthetic data until hardware is connected):

```bash
ros2 run x650_evaluation x650_synthetic_session data/synthetic/loop --motion loop --duration 20 --loop-closure 12:0.3
ros2 run x650_evaluation x650_timestamp_audit data/synthetic/loop/bag --stereo /rig/cam0/image_raw,/rig/cam1/image_raw --imu /rig/imu
ros2 run x650_evaluation x650_replay data/synthetic/loop/bag --out data/replays/loop \
    --reference-topic /groundtruth/odometry --sequence-type handheld_closed_loop
ros2 run x650_recording x650_record --sequence-type stationary --calibration calibration/rigs/<rig>/<id>   # real sessions
ros2 run x650_description calibration_hash calibration/rigs/<rig>/<new_id> --write   # new calibration
```

PX4 SITL (M3; PX4 lives outside the workspace in ~/px4):

```bash
scripts/setup_px4_sitl.sh                      # PX4 v1.17.0 + Gazebo Harmonic, builds px4_sitl
vcs import src/third_party < dependencies.repos && colcon build --symlink-install   # px4_msgs, XRCE agent
ros2 launch x650_sim sitl_vio.launch.py        # headless sim; fly with: ros2 run x650_sim x650_sim_mission
ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_faults    # automated test + report.json
```

USB webcam (bench camera until the ZED X works on this carrier):

```bash
v4l2-ctl --list-devices && v4l2-ctl -d /dev/video0 --list-formats-ext   # pick a YUYV mode
ros2 launch x650_camera usb_webcam.launch.py   # /rig/cam0/image_raw + /rig/cam0/frame_metadata
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
