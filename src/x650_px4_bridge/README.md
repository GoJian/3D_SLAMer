# x650_px4_bridge

**Milestone:** M3

**Responsibility:** Frame/time conversion and external-odometry transport to PX4 (ROS 2 `VehicleOdometry` or MAVLink `ODOMETRY`, chosen after pinning PX4).

**Main output:** PX4 external-vision input.

## Notes

Add `px4_msgs` (from `dependencies.repos`, branch matched to the pinned PX4 release) once the transport is chosen. Write transform/time conversion tests BEFORE connecting to PX4 (sprint item 5). Validate in SITL first.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
