# x650_model_viewer

Offline workstation tool. Lives outside the ROS workspace and uses its own environment, separate from flight software (plan section 15).

**Milestone:** M6

**Responsibility:** Orbit/fly/walk controls, surface picking against the reference mesh, distance/area tools, coverage overlays.

**Main output:** Navigable model and saved measurements.

## Notes

Technology stack (desktop vs web/three.js, etc.) is not yet chosen. Picking must use the versioned reference mesh, not the display LOD. Walk-mode collision is a viewer convenience only.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
