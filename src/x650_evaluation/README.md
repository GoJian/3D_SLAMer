# x650_evaluation

**Milestone:** M1+

**Responsibility:** Alignment, metrics, runtime reports, fault injection.

**Main output:** Machine-readable results (`reports/*.json`).

## Notes

SE(3) alignment for metric systems; report Sim(3) separately. Report ATE/RPE, latency p50/p95/p99, frame loss, queue age, memory, power, temperature. Report failures; never exclude them.

## Implemented (run with `ros2 run x650_evaluation <tool>`)

- `x650_synthetic_session`: synthetic MCAP session with ground truth and injected faults
  (dropped/late/offset stereo frames, IMU gaps, loop-closure shifts, estimator resets).
  Synthetic sessions test tooling only; they are not evidence of system performance.
- `x650_timestamp_audit`: per-topic rate/gaps/duplicates/reordering, stereo pairing offsets,
  IMU gaps, FrameMetadata sequence drops, record-minus-stamp latency.
- `x650_evaluate`: JSON report: ATE (SE(3) and Sim(3) reported separately), RPE, closed-loop drift,
  rate, latency percentiles, adapter health, plan section 12 gates.
- `x650_replay`: bag -> estimator adapter (sim time) -> recorded outputs -> report.

See `docs/3D_SLAMer_TechnicalPlan.md` sections 6 and 11.
