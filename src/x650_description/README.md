# x650_description

**Milestone:** M0/M2

**Responsibility:** URDF, calibrated transforms, mounting reference.

**Main output:** Static TF (`base_link`, `imu_link`, `cam_i_optical`).

## Notes

Transforms are loaded from versioned calibration files in `calibration/`, never hand-typed. Notation: `T_A_B` maps coordinates from B into A.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
