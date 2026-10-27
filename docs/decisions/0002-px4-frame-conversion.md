# 0002: ROS -> PX4 frame conversion defaults to FRD world (POSE_FRAME_FRD)

- Status: accepted; transport chosen in [0004](0004-px4-transport-uxrce-dds.md); verified in PX4 v1.17.0 SITL
- Date: 2026-10-27
- Milestone: M3 (implemented ahead as sprint item 5)

## Context

The VIO `odom` frame is gravity-aligned (z up) but its heading is wherever the estimator started.
It is not geographic ENU. Applying the textbook ENU -> NED swap (x<->y, z flip) to such a frame
reports a heading 90 deg off and mirrors x/y relative to the vehicle's start pose.

## Decision

- `x650_px4_bridge::WorldConversion::kZUpToFrd` (default): rotate 180 deg about x
  (x, -y, -z). Send with PX4 `POSE_FRAME_FRD` (arbitrary heading, z down).
- `kEnuToNed`: only when the ROS world frame really is ENU (e.g. after GNSS heading alignment);
  send with `POSE_FRAME_NED`.
- Body: FLU -> FRD (x, -y, -z). Velocity is sent in body FRD (`VELOCITY_FRAME_BODY_FRD`).
- `timestamp_sample` is the image acquisition time mapped to the PX4 clock, never publish time.
- Estimator resets increment the PX4 `reset_counter` (superseded by 0004: resets are absorbed
  upstream and `reset_counter` stays constant); stale (>100 ms default), future-dated,
  non-monotonic and non-finite samples are dropped and surfaced via `lastAcceptedAge`.
- Lever arm from base_link to the PX4 body reference is applied in the bridge
  (`T_base_px4body`); EKF2_EV_POS_X/Y/Z are then zero. Do not apply it twice.
- The core is transport-agnostic (`ExternalVisionSample`). The uXRCE-DDS `VehicleOdometry` vs
  MAVLink `ODOMETRY` node is written in M3 after the PX4 release is pinned.

## Consequences

Must be re-verified against the pinned PX4 release's `VehicleOdometry` definition and EKF2
external-vision configuration in SITL (M3 exit criterion).

## Evidence

SITL (0004): with a 0.6 rad estimator heading offset PX4 reported heading 0.600 rad (FRD sign
flip of z-up yaw), and EKF output matched the sent odometry without alignment (4 cm RMS).

`src/x650_px4_bridge/test/test_frame_conversion.cpp`: round trips, known-axis motions (climb is
-z, left is -y, left yaw is negative FRD yaw, ENU nose-east is NED yaw +90 deg), lever arm.
