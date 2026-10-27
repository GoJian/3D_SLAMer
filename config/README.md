# config

Project-wide configuration. JSON Schemas in `schemas/` are the contract; YAML files are instances.
Validate with `scripts/validate_config.py`.

| File | Schema | Purpose |
| --- | --- | --- |
| `hardware/x650_rev0.yaml` | `hardware_revision.schema.json` | Physical build record (M0) |
| `clock_policy.yaml` | `clock_policy.schema.json` | Sensor time -> ROS time policy |
| `dependencies.jetson.lock.yaml` | `dependency_lock.schema.json` | Pinned onboard stack (M0) |
| `dependencies.workstation.lock.yaml` | `dependency_lock.schema.json` | Pinned post-flight stack (M5) |

Other schemas describe runtime artifacts: `session_manifest` (recordings), `keyframe_bundle`
(persisted keyframes), `calibration` (`calibration/`), `model_manifest` (delivered `model/` package).

ROS node parameters live in each package's `config/` and are composed by `src/x650_bringup`.
