# /rig/cam_i/frame_metadata
# Per-frame acquisition metadata published alongside each image (same header.stamp).
# ROS 2 image headers carry no sequence number or exposure data; this message keeps them.
std_msgs/Header header              # stamp == matching Image header.stamp (acquisition, ROS time)
string camera_id
uint64 sequence                     # sensor frame counter; gaps = dropped frames
int64 sensor_stamp_ns               # original sensor-clock acquisition time
string clock_domain                 # e.g. "zed_camera" (config/clock_policy.yaml)
string stamp_reference              # "exposure_start" | "exposure_mid" | "readout_end"
float32 exposure_us
float32 analog_gain_db
float32 digital_gain
