// Time-domain utilities. All times are int64 nanoseconds unless the name says otherwise.
//
// Rule (CLAUDE.md): never use callback arrival time as acquisition time. Arrival time is used here
// only to estimate the offset between the sensor clock and the host clock.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

namespace x650_common
{

using Nanoseconds = std::int64_t;

// Exposure midpoint from an exposure-start stamp and exposure duration.
// Use when config/clock_policy.yaml says the sensor reports exposure_start but consumers need the
// midpoint (or vice versa: start = mid - exposure/2).
inline Nanoseconds exposureMidpoint(Nanoseconds exposure_start, Nanoseconds exposure_duration)
{
  return exposure_start + exposure_duration / 2;
}

// Estimates offset = host_clock - sensor_clock from (sensor_stamp, host_arrival) pairs.
//
// Model: arrival = sensor + offset + latency, latency >= 0. The minimum of (arrival - sensor) over
// a sliding window bounds the offset from above by the minimum transport latency. With clock drift
// d (ppm), the window length W adds at most d * W error (50 ppm over 2 s = 100 us).
//
// A sensor stamp that goes backwards, or jumps forward by more than reset_jump, is treated as a
// sensor clock reset: the window is cleared and reset_count() increments.
class ClockOffsetEstimator
{
public:
  struct Config
  {
    Nanoseconds window{2'000'000'000};        // sliding window length in sensor time
    std::size_t min_samples{5};               // samples needed before toHost() returns a value
    Nanoseconds reset_jump{1'000'000'000};    // forward jump treated as a clock reset
  };

  ClockOffsetEstimator() = default;
  explicit ClockOffsetEstimator(const Config & config)
  : config_(config) {}

  // Add one observation. Returns false if this sample triggered a reset.
  bool addSample(Nanoseconds sensor_stamp, Nanoseconds host_arrival)
  {
    bool ok = true;
    if (!samples_.empty()) {
      const Nanoseconds last = samples_.back().first;
      if (sensor_stamp < last || sensor_stamp - last > config_.reset_jump) {
        samples_.clear();
        ++reset_count_;
        ok = false;
      }
    }
    // Keep a monotonic deque of candidate minima (classic sliding-window minimum).
    const Nanoseconds diff = host_arrival - sensor_stamp;
    while (!samples_.empty() && samples_.back().second >= diff) {
      samples_.pop_back();
    }
    samples_.emplace_back(sensor_stamp, diff);
    while (samples_.front().first < sensor_stamp - config_.window) {
      samples_.pop_front();
    }
    ++count_since_reset_;
    if (!ok) {
      count_since_reset_ = 1;
    }
    return ok;
  }

  bool ready() const {return count_since_reset_ >= config_.min_samples && !samples_.empty();}

  // offset = host - sensor, or nullopt before enough samples have been seen.
  std::optional<Nanoseconds> offset() const
  {
    if (!ready()) {
      return std::nullopt;
    }
    return samples_.front().second;
  }

  // Map a sensor acquisition stamp into the host clock domain.
  std::optional<Nanoseconds> toHost(Nanoseconds sensor_stamp) const
  {
    const auto off = offset();
    if (!off) {
      return std::nullopt;
    }
    return sensor_stamp + *off;
  }

  std::uint64_t reset_count() const {return reset_count_;}

private:
  Config config_{};
  std::deque<std::pair<Nanoseconds, Nanoseconds>> samples_;  // (sensor_stamp, arrival - sensor)
  std::size_t count_since_reset_{0};
  std::uint64_t reset_count_{0};
};

// Gate for estimates passed downstream (e.g. to PX4). Rejects stale, future-dated, non-finite
// and non-monotonic samples. Does not update its "last accepted" stamp on rejection.
class StalenessGate
{
public:
  enum class Result { kOk, kStale, kFuture, kNonMonotonic };

  StalenessGate(Nanoseconds max_age, Nanoseconds max_future)
  : max_age_(max_age), max_future_(max_future) {}

  Result check(Nanoseconds sample_stamp, Nanoseconds now)
  {
    if (has_last_ && sample_stamp <= last_accepted_) {
      return Result::kNonMonotonic;
    }
    if (now - sample_stamp > max_age_) {
      return Result::kStale;
    }
    if (sample_stamp - now > max_future_) {
      return Result::kFuture;
    }
    last_accepted_ = sample_stamp;
    has_last_ = true;
    return Result::kOk;
  }

  // Call after an upstream estimator reset so a restarted time base is accepted.
  void reset() {has_last_ = false;}

  // Age of the last accepted sample, or nullopt if none.
  std::optional<Nanoseconds> lastAge(Nanoseconds now) const
  {
    if (!has_last_) {
      return std::nullopt;
    }
    return now - last_accepted_;
  }

private:
  Nanoseconds max_age_;
  Nanoseconds max_future_;
  Nanoseconds last_accepted_{0};
  bool has_last_{false};
};

inline const char * toString(StalenessGate::Result r)
{
  switch (r) {
    case StalenessGate::Result::kOk: return "ok";
    case StalenessGate::Result::kStale: return "stale";
    case StalenessGate::Result::kFuture: return "future";
    case StalenessGate::Result::kNonMonotonic: return "non_monotonic";
  }
  return "unknown";
}

// Tracks frame sequence numbers: dropped (gaps), duplicated and out-of-order frames.
class SequenceTracker
{
public:
  enum class Result { kFirst, kOk, kGap, kDuplicate, kOutOfOrder };

  Result update(std::uint64_t seq)
  {
    ++received_;
    if (!has_last_) {
      has_last_ = true;
      last_ = seq;
      return Result::kFirst;
    }
    if (seq == last_) {
      ++duplicates_;
      return Result::kDuplicate;
    }
    if (seq < last_) {
      ++out_of_order_;
      return Result::kOutOfOrder;
    }
    const std::uint64_t missing = seq - last_ - 1;
    last_ = seq;
    if (missing > 0) {
      dropped_ += missing;
      ++gap_events_;
      return Result::kGap;
    }
    return Result::kOk;
  }

  std::uint64_t received() const {return received_;}
  std::uint64_t dropped() const {return dropped_;}
  std::uint64_t gap_events() const {return gap_events_;}
  std::uint64_t duplicates() const {return duplicates_;}
  std::uint64_t out_of_order() const {return out_of_order_;}

private:
  bool has_last_{false};
  std::uint64_t last_{0};
  std::uint64_t received_{0};
  std::uint64_t dropped_{0};
  std::uint64_t gap_events_{0};
  std::uint64_t duplicates_{0};
  std::uint64_t out_of_order_{0};
};

// Convert a host-clock stamp into PX4 microseconds given offset = host - px4 (ns), as provided by
// the selected transport's timesync. Rounds to the nearest microsecond.
inline std::int64_t hostToPx4Micros(Nanoseconds host_stamp, Nanoseconds host_minus_px4)
{
  const Nanoseconds px4_ns = host_stamp - host_minus_px4;
  return (px4_ns >= 0 ? px4_ns + 500 : px4_ns - 500) / 1000;
}

inline Nanoseconds px4MicrosToHost(std::int64_t px4_us, Nanoseconds host_minus_px4)
{
  return px4_us * 1000 + host_minus_px4;
}

}  // namespace x650_common
