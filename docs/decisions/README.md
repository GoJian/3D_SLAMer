# Decisions

Record each decision as `NNNN-short-title.md` using `template.md`. Keep superseded records; mark them superseded.

## Open decisions (plan section 13)

Before procurement:

- [ ] Flight controller (Pixhawk 6C / 6X / Cube Orange)
- [ ] Battery
- [ ] Payload mounting
- [ ] Camera lens / FOV
- [ ] Jetson carrier and ZED Link capture support
- [ ] Regulated power and cooling

Before flight:

- [ ] Pin firmware and ROS interfaces
- [ ] PX4 transport: ROS 2 `VehicleOdometry` vs MAVLink `ODOMETRY`
- [ ] GNSS vs vision fusion policy
- [ ] Mission speed, distance and clearance
- [ ] Failure handling validated

Before distributing a product:

- [ ] Audit all dependency and model-weight licenses (Photo-SLAM / Gaussian-Splatting restrictions)
- [ ] Choose this repository's license
- [ ] Accuracy tolerance and coverage targets for the model product
