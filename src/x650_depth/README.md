# x650_depth

**Milestone:** M4

**Responsibility:** Stereo/SDK depth conversion, confidence and invalid masks.

**Main output:** `/stereo/depth` (32FC1 meters, invalid explicitly defined), `/stereo/confidence`.

## Notes

Record the vendor confidence scale and the conversion policy. Invalid depth must never clear obstacles.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
