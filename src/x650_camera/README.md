# x650_camera

**Milestone:** M1

**Responsibility:** Camera acquisition with acquisition metadata. Now: USB webcams (V4L2) for bench work. Later: ZED X / ZED SDK wrapper with images and IMU.

**Main output:** `/rig/cam_i/image_raw`, `/rig/cam_i/camera_info`, `/rig/imu`.

## USB webcam (`usb_camera_node`)

```bash
v4l2-ctl --list-devices; v4l2-ctl -d /dev/video0 --list-formats-ext
ros2 launch x650_camera usb_webcam.launch.py cam0_device:=/dev/video0 [cam1_device:=/dev/video2] [exposure_us:=8000]
```

- Publishes `/rig/cam_i/image_raw` (`mono8` from YUYV or GREY, or `yuv422_yuy2` passthrough) and
  `/rig/cam_i/frame_metadata`. MJPEG is not decoded; pick a YUYV mode.
- `header.stamp` is the kernel's V4L2 buffer timestamp (CLOCK_MONOTONIC, start of frame for
  UVC) mapped to CLOCK_REALTIME, not callback time. `FrameMetadata` carries the raw monotonic
  stamp (`clock_domain: host_monotonic`), what the stamp refers to, the V4L2 sequence number
  (gaps = dropped frames) and the exposure if the camera reports it.
- `/diagnostics` reports frames, drops, corrupt buffers and capture-to-dequeue age.
- Limits: two webcams are free-running, so they don't make a synchronized stereo pair (the
  timestamp audit shows the offset). No IMU, so no VIO from a webcam alone. No CameraInfo until a
  calibration exists. Auto exposure stays on unless `exposure_us` is set.
- Status: unit-tested (`test/test_v4l2_timing.cpp`); not yet run on a real device.

## Notes

External dependency: ZED SDK + ZED Link GMSL2 driver (not available via rosdep; pin in `config/dependencies.jetson.lock.yaml`; support on the J401 carrier is unconfirmed, see decision 0001). Keep acquisition timestamps, clock domain, sequence numbers and exposure metadata. Raw and rectified images use distinct topics. Never undistort twice. Publish `x650_msgs/FrameMetadata` on `/rig/cam_i/frame_metadata` per frame (sequence, sensor stamp, exposure); map sensor time with `x650_common::ClockOffsetEstimator`.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
