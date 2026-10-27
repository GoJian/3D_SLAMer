# x650_estimator_adapter

**Milestone:** M2

**Responsibility:** Normalize estimator (OKVIS2-X baseline; cuVSLAM alternative) output, health, covariance, reset and global-correction behavior.

**Main output:** `/vio/odometry` (odom -> base_link), `/vio/health`, `map -> odom` TF, `/mapping/pose_updates`.

## Notes

One authoritative pose producer per output stream. Loop-closure jumps go into `map -> odom`, never into continuous local odometry.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
