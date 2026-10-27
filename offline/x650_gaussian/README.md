# x650_gaussian

Offline workstation tool. Lives outside the ROS workspace and uses its own environment, separate from flight software (plan section 15).

**Milestone:** M9 (optional)

**Responsibility:** Offline Gaussian comparison first, then a bounded asynchronous submap worker.

**Main output:** Gaussian artifacts + mapper health. Never defines the measurement surface.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
