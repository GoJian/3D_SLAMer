# x650_common

**Milestone:** M1–M3 (sprint item 5)

Header-only C++17 utilities with no ROS runtime dependency (Eigen only):

- `rigid.hpp`: `Pose` (`T_A_B`), body-frame `Odometry`, covariance rotation, lever-arm change.
- `time_mapping.hpp`: `ClockOffsetEstimator` maps sensor acquisition time into the host/ROS clock
  domain. Callback arrival time is used only to *estimate* the offset (minimum-latency filter); it
  never replaces the acquisition stamp. Detects sensor clock resets.
- `StalenessGate`: rejects stale, future-dated and non-monotonic samples.
- `SequenceTracker`: counts dropped, duplicated and reordered frames from sequence numbers.

All times are signed 64-bit nanoseconds unless a name says otherwise.
