# x650_keyframes

**Milestone:** M5/M9

**Responsibility:** Select and persist synchronized views and pose revisions.

**Main output:** Keyframe manifest on disk + `/mapping/keyframe` events.

## Notes

Bundles follow `config/schemas/keyframe_bundle.schema.json`. Events reference persisted files; do not send full-resolution images in high-rate custom messages.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
