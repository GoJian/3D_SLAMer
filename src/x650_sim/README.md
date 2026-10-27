# x650_sim

**Milestone:** M3

**Responsibility:** PX4 SITL harness for the external-vision chain: a fake VIO with fault
injection, an offboard test mission, an automated flight test and its evaluation.

## Prerequisites

`scripts/setup_px4_sitl.sh` installs PX4 v1.17.0 SITL and Gazebo Harmonic into `~/px4` (or
`$PX4_DIR`). `px4_msgs` and the Micro-XRCE-DDS-Agent come from `dependencies.repos`
(decision 0004).

## Use

```bash
ros2 launch x650_sim sitl_vio.launch.py                 # Gazebo (headless) + PX4 + agent + chain
ros2 run x650_sim x650_sim_mission --ros-args -p use_sim_time:=true -p faults:='1:jump 0.5'
ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_faults     # all of the above + report
ros2 run x650_sim x650_sim_evaluate data/sim/m3_faults/bag         # re-evaluate a recording
```

Data path: Gazebo odometry -> `fake_vio` -> `/estimator/odometry` -> `estimator_adapter` ->
`/vio/odometry` -> `px4_bridge` -> `/fmu/in/vehicle_visual_odometry` -> EKF2 (no GPS, no mag).

- `fake_vio` faults on `/sim/fault`: `dropout <s>`, `jump <m>`, `reset`,
  `latency <s> [duration_s]`. The estimator world starts at the first pose with a heading offset
  (`yaw_offset_rad`, default 0.6) as a real VIO would.
- `time_source:=sim` (default): ROS on Gazebo `/clock`, `UXRCE_DDS_SYNCT=0`.
  `time_source:=wall`: host clock with uXRCE timesync, the hardware configuration.
- PX4 parameters: `config/px4_sitl_ev_params.yaml`. They are applied as `PX4_PARAM_*` at boot and
  again with `px4-param` after boot (see decision 0004 for why both).
- Gates (`sim_eval.GATES`): axes, timing, accuracy, local continuity at jumps/resets, stale
  rejection, EV fusion outside planned outages, resets counted, mission completed.

The fake VIO is ground truth plus noise. These tests check integration and timing, not estimator
performance. No camera simulation: Gazebo runs without rendering because the GPU doesn't
initialize on this Jetson image.
