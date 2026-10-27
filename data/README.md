# data (git-ignored)

Local storage for recordings, keyframe bundles, reconstruction work dirs and model outputs.
Nothing in here is committed. On the Jetson, point this at NVMe (symlink is fine).

Suggested layout:

```text
data/sessions/<session_id>/      # rosbag2 (MCAP) + session_manifest.json
data/keyframes/<session_id>/     # keyframe bundles
data/reconstruction/<run_id>/    # COLMAP / OpenMVS work dirs
data/models/<model_id>/          # delivered model/ packages
```
