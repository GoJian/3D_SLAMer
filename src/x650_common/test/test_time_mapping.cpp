#include <gtest/gtest.h>

#include <random>

#include "x650_common/time_mapping.hpp"

using x650_common::ClockOffsetEstimator;
using x650_common::Nanoseconds;
using x650_common::SequenceTracker;
using x650_common::StalenessGate;

namespace
{
constexpr Nanoseconds kMs = 1'000'000;
}

TEST(ExposureMidpoint, AddsHalfExposure)
{
  EXPECT_EQ(x650_common::exposureMidpoint(1000, 400), 1200);
}

TEST(ClockOffsetEstimator, NotReadyBeforeMinSamples)
{
  ClockOffsetEstimator est;
  for (int i = 0; i < 4; ++i) {
    est.addSample(i * 33 * kMs, i * 33 * kMs + 5 * kMs);
  }
  EXPECT_FALSE(est.toHost(0).has_value());
  est.addSample(5 * 33 * kMs, 5 * 33 * kMs + 5 * kMs);
  ASSERT_TRUE(est.toHost(0).has_value());
}

// Constant offset plus random positive latency: estimate within the minimum latency.
TEST(ClockOffsetEstimator, RecoversOffsetUnderJitter)
{
  const Nanoseconds offset = 1'700'000'000'000'000'000LL;  // host epoch vs sensor boot time
  const Nanoseconds min_latency = 2 * kMs;
  std::mt19937 rng(42);
  std::exponential_distribution<double> jitter(1.0 / (3.0 * kMs));
  ClockOffsetEstimator est;
  for (int i = 0; i < 300; ++i) {
    const Nanoseconds sensor = 10'000 * kMs + i * 33 * kMs;
    const Nanoseconds arrival = sensor + offset + min_latency + static_cast<Nanoseconds>(jitter(rng));
    EXPECT_TRUE(est.addSample(sensor, arrival));
  }
  const auto off = est.offset();
  ASSERT_TRUE(off.has_value());
  EXPECT_GE(*off, offset + min_latency);
  EXPECT_LT(*off, offset + min_latency + 1 * kMs);
  // Mapping must use the acquisition stamp, not the arrival time.
  EXPECT_EQ(*est.toHost(123), 123 + *off);
}

// With drift, the sliding window bounds the error.
TEST(ClockOffsetEstimator, TracksDrift)
{
  const double drift = 50e-6;  // 50 ppm
  ClockOffsetEstimator est;
  Nanoseconds true_offset = 0;
  for (int i = 0; i < 30 * 30; ++i) {  // 30 s at 30 Hz
    const Nanoseconds sensor = i * 33 * kMs;
    true_offset = static_cast<Nanoseconds>(sensor * drift);
    est.addSample(sensor, sensor + true_offset + 1 * kMs);
  }
  const Nanoseconds err = *est.offset() - (true_offset + 1 * kMs);
  // window 2 s * 50 ppm = 100 us
  EXPECT_LE(std::llabs(err), 110'000);
}

TEST(ClockOffsetEstimator, DetectsBackwardsReset)
{
  ClockOffsetEstimator est;
  for (int i = 0; i < 10; ++i) {
    est.addSample(1000 * kMs + i * 33 * kMs, 1000 * kMs + i * 33 * kMs + 7 * kMs);
  }
  ASSERT_TRUE(est.ready());
  EXPECT_FALSE(est.addSample(0, 2000 * kMs));
  EXPECT_EQ(est.reset_count(), 1u);
  EXPECT_FALSE(est.ready());
}

TEST(ClockOffsetEstimator, DetectsForwardJumpReset)
{
  ClockOffsetEstimator est;
  est.addSample(0, 10);
  EXPECT_FALSE(est.addSample(5'000 * kMs, 5'000 * kMs + 10));
  EXPECT_EQ(est.reset_count(), 1u);
}

TEST(StalenessGate, AcceptsFreshRejectsStaleFutureAndRepeat)
{
  StalenessGate gate(50 * kMs, 5 * kMs);
  EXPECT_EQ(gate.check(1000 * kMs, 1010 * kMs), StalenessGate::Result::kOk);
  EXPECT_EQ(gate.check(1000 * kMs, 1020 * kMs), StalenessGate::Result::kNonMonotonic);
  EXPECT_EQ(gate.check(1001 * kMs, 1100 * kMs), StalenessGate::Result::kStale);
  EXPECT_EQ(gate.check(1200 * kMs, 1100 * kMs), StalenessGate::Result::kFuture);
  EXPECT_EQ(gate.check(1101 * kMs, 1110 * kMs), StalenessGate::Result::kOk);
  EXPECT_EQ(*gate.lastAge(1120 * kMs), 19 * kMs);
}

TEST(StalenessGate, ResetAllowsRestartedTimeBase)
{
  StalenessGate gate(50 * kMs, 5 * kMs);
  ASSERT_EQ(gate.check(1000 * kMs, 1000 * kMs), StalenessGate::Result::kOk);
  EXPECT_EQ(gate.check(500 * kMs, 1000 * kMs), StalenessGate::Result::kNonMonotonic);
  gate.reset();
  EXPECT_EQ(gate.check(990 * kMs, 1000 * kMs), StalenessGate::Result::kOk);
}

TEST(SequenceTracker, CountsGapsDuplicatesReorders)
{
  SequenceTracker t;
  EXPECT_EQ(t.update(10), SequenceTracker::Result::kFirst);
  EXPECT_EQ(t.update(11), SequenceTracker::Result::kOk);
  EXPECT_EQ(t.update(14), SequenceTracker::Result::kGap);
  EXPECT_EQ(t.update(14), SequenceTracker::Result::kDuplicate);
  EXPECT_EQ(t.update(12), SequenceTracker::Result::kOutOfOrder);
  EXPECT_EQ(t.dropped(), 2u);
  EXPECT_EQ(t.gap_events(), 1u);
  EXPECT_EQ(t.duplicates(), 1u);
  EXPECT_EQ(t.out_of_order(), 1u);
  EXPECT_EQ(t.received(), 5u);
}

TEST(Px4Time, RoundTripAndRounding)
{
  const Nanoseconds host_minus_px4 = 1'700'000'000'000'000'000LL;
  const Nanoseconds host = host_minus_px4 + 12'345'678'901;  // 12.345678901 s after PX4 boot
  EXPECT_EQ(x650_common::hostToPx4Micros(host, host_minus_px4), 12'345'679);
  EXPECT_EQ(x650_common::px4MicrosToHost(12'345'679, host_minus_px4), host_minus_px4 + 12'345'679'000);
}
