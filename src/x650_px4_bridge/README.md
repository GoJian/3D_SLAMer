# x650_px4_bridge

**Milestone:** M3

**Responsibility:** Frame/time conversion and external-odometry transport to PX4 over uXRCE-DDS `VehicleOdometry` (decision 0004, PX4 v1.17.0).

**Main output:** PX4 external-vision input.

## Implemented

- `px4_bridge_node` (component `x650_px4_bridge::Px4BridgeNode`, config `config/px4_bridge.yaml`):
  `/vio/odometry` -> `/fmu/in/vehicle_visual_odometry`. `timestamp_sample` = acquisition time on the
  host clock; PX4's uXRCE-DDS client converts it (so `EKF2_EV_DELAY=0`). Sends only while
  `/fmu/out/timesync_status` is fresh (`require_timesync`, false in SITL on sim time). Republishes
  PX4's EKF output as `/px4/odometry` (REP-103) and its status on `/px4_bridge/status`.
- Verified in PX4 v1.17.0 SITL with fault injection: `src/x650_sim`, `reports/2026-10-27_m3_sitl.md`.

- `frame_conversion.hpp`: ROS (z-up world, FLU body) <-> PX4 (FRD or NED world, FRD body) for
  pose, body velocity/rates and covariances. Default world mapping is FRD (decision 0002).
- `external_vision.hpp`: `ExternalVisionBuilder` fills PX4 external-vision fields from local
  odometry: acquisition-time `timestamp_sample`, staleness/future/monotonic/NaN gating,
  quality 0..100, optional lever arm to the PX4 body point. The node keeps `reset_counter`
  constant because `/vio/odometry` is continuous (decisions 0003, 0004).
- Tests: `test/test_frame_conversion.cpp`, `test/test_external_vision.cpp`.

Requires `px4_msgs` release/1.17 from `dependencies.repos`. A different PX4 release needs the
matching branch.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
