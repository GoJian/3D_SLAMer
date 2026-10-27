# x650_description

**Milestone:** M0/M2

**Responsibility:** URDF, calibrated transforms, mounting reference.

**Main output:** Static TF (`base_link`, `imu_link`, `cam_i_optical`).

## Notes

Transforms are loaded from versioned calibration files in `calibration/`, never hand-typed. Notation: `T_A_B` maps coordinates from B into A.

## Implemented

- `x650_description.calibration`: load + schema-validate `calibration.yaml`, verify its content
  hash (edits in place are rejected), produce static transforms.
- `calibration_tf_publisher` / `launch/description.launch.py calibration:=<dir>`: static TF
  `base_link -> imu_link -> cam_i_optical`.
- `calibration_hash <new_calibration_dir> --write`: fill in the hash of a new calibration.
- Synthetic test fixture: `test/fixtures/example_rig/` (not a real calibration).

Not yet: URDF/meshes (needs measured mounting geometry).

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
