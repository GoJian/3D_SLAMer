# x650_model_export

Offline workstation tool. Lives outside the ROS workspace and uses its own environment, separate from flight software (plan section 15).

**Milestone:** M5/M6

**Responsibility:** Texture/geometry packaging, coordinate metadata, validation and display LODs.

**Main output:** `model/` package: GLB, reference mesh, point cloud, quality report (see `config/schemas/model_manifest.schema.json`).

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
