# Milestones and checklists

Source of truth: [`3D_SLAMer_TechnicalPlan.md`](3D_SLAMer_TechnicalPlan.md) section 11. Tick items as they land.
Start M0–M2 without waiting for Gaussian work. Every experiment must be reproducible from the same recordings.

## First implementation sprint

- [ ] 1. ROS 2 workspace with `x650_description`, `x650_recording`, `x650_camera`, `x650_estimator_adapter`, `x650_evaluation` (skeletons exist)
- [ ] 2. Config schemas for hardware revision, calibration hash, clock policy, dependency versions (`config/schemas/`; fill instances)
- [ ] 3. Record one stationary sequence and one handheld six-axis closed-loop sequence
- [ ] 4. Replay through the estimator; save trajectory, timing, initialization and reset diagnostics
- [ ] 5. Transform/time conversion tests before connecting the PX4 bridge
- [ ] 6. One machine-readable baseline report; freeze the recording as a regression fixture (`tests/fixtures/`)

## M0 — Dependency lock

Work: record hardware SKUs and exact PX4, JetPack, ROS, SDK and estimator revisions.

- [ ] `config/hardware/x650_rev0.yaml` filled
- [ ] `config/dependencies.jetson.lock.yaml` pinned (use `scripts/capture_env.sh`)
- [ ] `dependencies.repos` versions pinned to tags/commits
- [ ] Exit: camera, IMU and bridge smoke tests pass independently on target hardware

## M1 — Record / replay

Work: camera wrapper, timestamp audit, session manifest, calibration loader.

- [ ] `x650_camera` publishes images, CameraInfo, IMU with acquisition timestamps
- [ ] `x650_recording` writes MCAP + `session_manifest.json`
- [ ] Timestamp audit tool in `x650_evaluation`
- [ ] Exit: repeatable replay; frame pairing and timing verified; sustained recording without unexplained loss

## M2 — VIO baseline

Work: estimator adapter, TF, quality and reset reporting.

- [ ] Kalibr calibration in `calibration/rigs/`
- [ ] `x650_estimator_adapter` publishes `/vio/odometry`, `/vio/health`, `map -> odom -> base_link`
- [ ] Exit: metric scale and axis checks pass; closed-loop handheld trajectories evaluated

## M3 — PX4 SITL

Work: odometry bridge, simulated dropouts/resets/latency.

- [ ] Choose transport (ROS 2 `VehicleOdometry` vs MAVLink `ODOMETRY`) after pinning PX4
- [ ] Exit: correct axes and timing; stale-data behavior works; global correction does not jump local odometry

## M4 — Geometry

Work: confidence depth adapter and local map.

- [ ] Exit: known obstacles, unknown regions and depth-range limits validated

## M5 — Final metric model

Work: refined trajectory/rig export, dense reconstruction, mesh and texture pipeline (workstation).

- [ ] Exit: point cloud and textured mesh share a metric frame; independent accuracy and coverage report

## M6 — Navigable model

Work: GLB packaging, reference-geometry picking, measurement tools, quality overlays.

- [ ] Exit: known-distance tests pass; display LOD does not change reference measurements; invalid regions flagged

## M7 — Flight validation

Work: ground motor tests, controlled hover, incremental mission tests.

- [ ] Exit: stable estimates under real vibration; logs reviewed before expanding the flight envelope

## M8 — Second stereo rig

Work: camera IDs, multi-camera acquisition, extrinsics, bundle selection.

- [ ] Exit: physical synchronization verified; measurable coverage benefit under fixed compute budget

## M9 — Optional Gaussian mapping

Work: offline comparison, then bounded onboard worker.

- [ ] Exit: mesh product remains independent; navigation latency, memory and queues stay bounded

## Initial gates (plan section 12)

- >= 30 Hz visual updates; p95 capture-to-odometry age < 50 ms
- No unhandled resets over the target mission; bounded memory and queues
- Loss-of-vision handling works
- Required integration tests: transform round trips, known-axis motions, timestamp conversion, missing/late
  stereo frames, IMU gaps, estimator resets, global pose corrections, invalid depth, full storage, mapper crash,
  memory pressure, replay consistency
