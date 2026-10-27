# x650_geometry

**Milestone:** M4

**Responsibility:** Local geometric fusion (nvblox or comparable) and 3D distance/occupancy queries.

**Main output:** TSDF / occupancy / ESDF with explicit unknown space.

## Notes

nvblox / isaac_ros_nvblox are external (see `dependencies.repos`). Aerial planning needs 3D queries, not only a 2D costmap. Must keep running if Gaussian mapping is disabled or crashes.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
