# 0004: PX4 v1.17.0, external vision over uXRCE-DDS `VehicleOdometry`

- Status: accepted for SITL; to be re-confirmed on the flight controller hardware
- Date: 2026-10-27
- Milestone: M3

## Context

M3 needs a pinned PX4 release and a transport for external vision. The flight controller is not
chosen yet. Options: ROS 2 `px4_msgs/VehicleOdometry` over the uXRCE-DDS bridge built into PX4, or
MAVLink `ODOMETRY` via MAVROS.

## Decision

- PX4 **v1.17.0** (latest stable on 2026-10-27), `px4_msgs` **release/1.17** at `86d8239`,
  Micro-XRCE-DDS-Agent **v2.4.3** (PX4 v1.17 docs: the Jazzy-compatible agent). Pins in
  `dependencies.repos` and `config/dependencies.jetson.lock.yaml`.
- Transport: **uXRCE-DDS**. `x650_px4_bridge/px4_bridge_node` publishes
  `/fmu/in/vehicle_visual_odometry`. No MAVROS in the flight path.
- Timing: `timestamp_sample` is the image acquisition time on the **host clock** in microseconds.
  PX4's uXRCE-DDS client converts `timestamp` and `timestamp_sample` of incoming messages from
  agent time to PX4 time itself (`UXRCE_DDS_SYNCT=1`, Tools/msg/templates/ucdr/msg.h.em), so the
  bridge applies no offset and `EKF2_EV_DELAY` is **0**. The bridge only sends while
  `/fmu/out/timesync_status` is fresh (`require_timesync`).
- Outgoing PX4 topics arrive already on the host clock; `/fmu/out/vehicle_odometry` is republished
  in REP-103 as `/px4/odometry` for evaluation.
- `reset_counter` stays constant: `/vio/odometry` is continuous by contract (decision 0003), so
  estimator resets never reach PX4 as discontinuities. This supersedes the reset bullet of 0002.
- EKF2 for vision-only flight: `EKF2_EV_CTRL=15`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0`,
  `EKF2_MAG_TYPE=5`, `SYS_HAS_GPS=0`, `SYS_HAS_MAG=0` (`src/x650_sim/config/px4_sitl_ev_params.yaml`).
  With `POSE_FRAME_FRD`, EKF2 deliberately keeps `yaw_align=false`, so
  `vehicle_local_position.heading_good_for_control` stays false. Only manual attitude-mode
  heading hold uses that flag; position and offboard control are unaffected.
- SITL time source: ROS nodes run on Gazebo `/clock` with `UXRCE_DDS_SYNCT=0` (PX4's documented
  alternative for Gazebo), so PX4 time equals ROS time and no conversion is involved. Wall-clock
  mode with timesync remains available (`time_source:=wall`) and is the hardware configuration.

## Consequences

- PX4 SITL is installed outside the workspace (`scripts/setup_px4_sitl.sh`, ~/px4). PX4 ships a
  `package.xml`, so it must not live under `src/`.
- SITL parameter trap: `PX4_PARAM_*` environment overrides run before the airframe's
  `param set-default`. A value equal to the old default is not marked as user-set and can be
  changed by the airframe (x500 sets `NAV_DLL_ACT 2`, which then blocks arming with "No
  connection to the GCS"). The launch re-applies all parameters with `px4-param` after boot.
- In lockstep SITL with wall-clock timesync, PX4's host-mapped IMU stamps were ~18 ms early and
  PX4's output ~15-20 ms late against Gazebo ground truth. On the sim clock the IMU offset stays
  ~16 ms (PX4's Gazebo bridge stamps IMU on arrival, GZBridge::imuCallback). That is internal to
  PX4 SITL: changing our vision latency from 0 to 80 ms did not change PX4's output lag (-10 ms
  both), so EKF2 compensates `timestamp_sample` correctly. Re-measure on hardware.
- After a ~1 s vision outage PX4 corrected its IMU-only drift by 16-18 cm when vision returned.
  The flight supervisor (M7) must treat vision loss as a position-quality event, not just a flag.
- Re-check this record when the flight controller and its PX4 build are chosen. A different PX4
  release needs a matching `px4_msgs` branch (versioned topics such as
  `vehicle_local_position_v1` change names).

## Evidence

- `reports/2026-10-27_m3_sitl.md`, with JSON reports from `data/sim/` runs (recordings not
  committed).
- `src/x650_sim` flight test: `ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_faults`.
