# x650_camera

**Milestone:** M1

**Responsibility:** Wrap vendor (ZED X / ZED SDK) acquisition, retain acquisition metadata, provide calibrated images and IMU.

**Main output:** `/rig/cam_i/image_raw`, `/rig/cam_i/camera_info`, `/rig/imu`.

## Notes

External dependency: ZED SDK + ZED Link GMSL2 driver (not available via rosdep; pin in `config/dependencies.lock.yaml`). Keep acquisition timestamps, clock domain, sequence numbers and exposure metadata. Raw and rectified images use distinct topics. Never undistort twice.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
