# Decisions

## Records

- [0001](0001-onboard-platform-jetpack7-jazzy.md) Onboard platform: JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy
- [0002](0002-px4-frame-conversion.md) ROS -> PX4 frame conversion defaults to FRD world
- [0003](0003-local-global-split.md) Estimator adapter: odom from increments, jumps into map -> odom (amended after SITL)
- [0004](0004-px4-transport-uxrce-dds.md) PX4 v1.17.0, external vision over uXRCE-DDS `VehicleOdometry`

Record each decision as `NNNN-short-title.md` using `template.md`. Keep superseded records; mark them superseded.

## Open decisions (plan section 13)

Before procurement:

- [ ] Flight controller (Pixhawk 6C / 6X / Cube Orange)
- [ ] Battery
- [ ] Payload mounting
- [ ] Camera lens / FOV
- [ ] Jetson carrier and ZED X GMSL support (carrier is reComputer Robotics J401, decision 0001; ZED X driver for it unconfirmed)
- [ ] Regulated power and cooling

Before flight:

- [ ] Pin firmware and ROS interfaces (PX4 v1.17.0 pinned for SITL, decision 0004; confirm on the chosen flight controller)
- [x] PX4 transport: ROS 2 `VehicleOdometry` over uXRCE-DDS (decision 0004)
- [ ] GNSS vs vision fusion policy
- [ ] Mission speed, distance and clearance
- [ ] Failure handling validated

Before distributing a product:

- [ ] Audit all dependency and model-weight licenses (Photo-SLAM / Gaussian-Splatting restrictions)
- [ ] Choose this repository's license
- [ ] Accuracy tolerance and coverage targets for the model product
