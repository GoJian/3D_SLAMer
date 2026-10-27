# 0003: Estimator adapter builds odom from increments; jumps go into map -> odom

- Status: accepted
- Date: 2026-10-27
- Milestone: M2

## Context

Estimators such as OKVIS2-X publish a pose in their own world frame that jumps on loop closure and
restarts on reset. Flight control needs continuous odometry (plan section 5).

## Decision

`x650_estimator_adapter::OdometrySplitter`:

- `T_odom_base` is integrated from estimator increments `T_E_B(k-1)^-1 * T_E_B(k)`.
- An update is a *jump* if the estimator flags a global correction, its reset counter changes, the
  gap exceeds `max_gap` (2 s), or the increment disagrees with the velocity-predicted step (mean of
  previous and current body twist) by more than 0.25 m + 0.25 m/s * dt / 0.20 rad + 0.20 rad/s * dt.
  Jump updates advance odom by that constant-twist step instead.
- A shorter estimator dropout is not a jump by itself; the increment across it is trusted.
- `T_map_odom = T_E_B * T_odom_base^-1` after every update, so the map-frame pose always equals the
  estimator's (corrected) pose. Jumps increment `pose_revision`.
- Input is generic `nav_msgs/Odometry` (IMU pose in estimator world) plus an optional
  `std_msgs/UInt64` reset counter, so any estimator can be wired with a thin remap.

## Consequences

- Thresholds are placeholders until tuned on real recordings; aggressive flight may need larger
  values. Report jump counts in every evaluation.
- Covariance in odom is the estimator's covariance rotated into odom; it is not inflated across
  jumps. Revisit when wiring PX4 EKF2 (M3).

## Amendment 2026-10-27 (M3 SITL)

The first version also treated every gap longer than 0.5 s as a jump. In PX4 SITL
(`data/sim/m3_faults`, not committed) a 1 s estimator dropout while the vehicle started a new leg
made odom discard 0.84 m of real motion. PX4 had integrated that motion with its IMU, so when
vision returned EKF2 reset its position by 0.94 m: the jump this split exists to prevent. Gaps up
to `max_gap` (2 s) are now judged by the same consistency test, with a tolerance that grows with
dt and a trapezoidal velocity prediction. Regression tests: `ShortDropoutKeepsRealMotion`,
`DropoutWhileAcceleratingIsNotAJump`, `LoopClosureAfterShortDropoutStillDetected`.

## Evidence

`src/x650_estimator_adapter/test/test_odometry_splitter.cpp`; replay integration test
`src/x650_evaluation/test/test_replay.py` (synthetic loop closure + reset: max odom step < 10 cm,
both absorbed, reset surfaced in health). Synthetic only; not evidence of estimator performance.
