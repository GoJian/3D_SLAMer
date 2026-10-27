# x650_estimator_adapter

**Milestone:** M2

**Responsibility:** Normalize estimator (OKVIS2-X baseline; cuVSLAM alternative) output, health, covariance, reset and global-correction behavior.

**Main output:** `/vio/odometry` (odom -> base_link), `/vio/health`, `map -> odom` TF, `/mapping/pose_updates`.

## Notes

One authoritative pose producer per output stream. Loop-closure jumps go into `map -> odom`, never into continuous local odometry.

## Implemented

- `OdometrySplitter` (header-only): continuous `T_odom_base` from estimator increments; loop
  closures, resets and long gaps go into `map -> odom` (decision 0003).
- `estimator_adapter_node` (component): `~/input/odometry` (nav_msgs/Odometry, IMU pose in the
  estimator world) + optional `~/input/reset_count` (std_msgs/UInt64) -> `/vio/odometry`, TF
  `map -> odom -> base_link`, `/vio/health` (DiagnosticArray at 10 Hz).
- `launch/estimator_adapter.launch.py calibration:=<dir>` reads `T_base_imu` from calibration.

Next (M2): import OKVIS2-X, remap its odometry output here, run on recordings.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
