# x650_recording

**Milestone:** M1

**Responsibility:** Capture raw inputs, calibration, configuration and diagnostics into a replayable session.

**Main output:** Session directory: rosbag2 (MCAP) + `session_manifest.json`.

## Notes

Manifests follow `config/schemas/session_manifest.schema.json`. Size storage from measured sustained throughput (plan section 10). Preserve original acquisition time in recordings.

## Implemented

- `ros2 run x650_recording x650_record --sequence-type <type> --calibration <dir>`: records MCAP
  (`config/record_topics.yaml`, bounded writer cache), snapshots calibration + config into the
  session, runs the timestamp audit and writes a schema-validated `session_manifest.json`.
- `--finalize-only` audits and writes the manifest for an already-recorded `<session>/bag`.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
