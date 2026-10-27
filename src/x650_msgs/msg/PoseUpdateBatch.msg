# /mapping/pose_updates
# Batch of keyframe poses changed by a global correction (e.g. loop closure).
std_msgs/Header header
string session_id
uint64 revision                     # monotonically increasing pose revision
KeyframePose[] poses
