# x650_recording

**Milestone:** M1

**Responsibility:** Capture raw inputs, calibration, configuration and diagnostics into a replayable session.

**Main output:** Session directory: rosbag2 (MCAP) + `session_manifest.json`.

## Notes

Manifests follow `config/schemas/session_manifest.schema.json`. Size storage from measured sustained throughput (plan section 10). Preserve original acquisition time in recordings.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
