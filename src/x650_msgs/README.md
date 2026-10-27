# x650_msgs

**Milestone:** M2+ (keyframes/pose updates), M9 (mapping health)

Project-defined messages for interfaces in plan section 6. Standard types are used wherever they fit
(`sensor_msgs/Image`, `nav_msgs/Odometry`, `diagnostic_msgs` for `/vio/health`).

| Topic | Message |
| --- | --- |
| `/mapping/keyframe` | `KeyframeEvent` |
| `/mapping/pose_updates` | `PoseUpdateBatch` |
| `/mapping/health` | `MappingHealth` |
