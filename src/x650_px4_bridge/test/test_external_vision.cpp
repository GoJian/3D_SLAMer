#include <gtest/gtest.h>

#include "x650_px4_bridge/external_vision.hpp"

using namespace x650_px4_bridge;
using x650_common::Nanoseconds;

namespace
{
constexpr Nanoseconds kMs = 1'000'000;
constexpr Nanoseconds kHostMinusPx4 = 1'000'000 * kMs;

LocalOdometryInput sample(Nanoseconds stamp, std::uint64_t resets = 0)
{
  LocalOdometryInput in;
  in.acquisition_stamp = stamp;
  in.odom.T_world_body.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  in.odom.position_covariance = Eigen::Vector3d(0.01, 0.04, 0.09).asDiagonal();
  in.estimator_reset_count = resets;
  in.quality = 0.8f;
  return in;
}
}  // namespace

TEST(ExternalVision, ConvertsAndStampsWithAcquisitionTime)
{
  ExternalVisionBuilder b({});
  ExternalVisionSample out;
  const Nanoseconds stamp = kHostMinusPx4 + 5'000 * kMs;
  ASSERT_EQ(b.build(sample(stamp), stamp + 20 * kMs, kHostMinusPx4, out),
    ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(out.timestamp_sample_us, 5'000'000);  // acquisition, not "now"
  EXPECT_FLOAT_EQ(out.position[0], 1.0f);
  EXPECT_FLOAT_EQ(out.position[1], -2.0f);
  EXPECT_FLOAT_EQ(out.position[2], -3.0f);
  EXPECT_FLOAT_EQ(out.position_variance[1], 0.04f);
  EXPECT_FLOAT_EQ(out.q[0], 1.0f);
  EXPECT_EQ(out.quality, 80);
}

TEST(ExternalVision, RejectsStaleFutureRepeatsNanAndUnsynced)
{
  ExternalVisionBuilder b({});
  ExternalVisionSample out;
  const Nanoseconds t = kHostMinusPx4 + 5'000 * kMs;
  EXPECT_EQ(b.build(sample(t), t, std::nullopt, out), ExternalVisionBuilder::Status::kNoTimesync);
  EXPECT_EQ(b.build(sample(t), t + 200 * kMs, kHostMinusPx4, out), ExternalVisionBuilder::Status::kStale);
  EXPECT_EQ(b.build(sample(t), t - 50 * kMs, kHostMinusPx4, out), ExternalVisionBuilder::Status::kFuture);
  ASSERT_EQ(b.build(sample(t), t, kHostMinusPx4, out), ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(b.build(sample(t), t + 1, kHostMinusPx4, out), ExternalVisionBuilder::Status::kNonMonotonic);
  auto bad = sample(t + 10 * kMs);
  bad.odom.linear_velocity_body.x() = std::nan("");
  EXPECT_EQ(b.build(bad, t + 10 * kMs, kHostMinusPx4, out), ExternalVisionBuilder::Status::kNonFinite);
}

TEST(ExternalVision, EstimatorResetIncrementsPx4ResetCounter)
{
  ExternalVisionBuilder b({});
  ExternalVisionSample out;
  const Nanoseconds t = kHostMinusPx4 + 5'000 * kMs;
  ASSERT_EQ(b.build(sample(t, 0), t, kHostMinusPx4, out), ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(out.reset_counter, 0);
  // After a reset the estimator may restart slightly earlier; it is accepted, counter bumps.
  ASSERT_EQ(b.build(sample(t - 1 * kMs, 1), t, kHostMinusPx4, out), ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(out.reset_counter, 1);
  ASSERT_EQ(b.build(sample(t + 33 * kMs, 1), t + 40 * kMs, kHostMinusPx4, out),
    ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(out.reset_counter, 1);
}

TEST(ExternalVision, LossOfVisionVisibleThroughLastAcceptedAge)
{
  ExternalVisionBuilder b({});
  ExternalVisionSample out;
  const Nanoseconds t = kHostMinusPx4 + 5'000 * kMs;
  EXPECT_FALSE(b.lastAcceptedAge(t).has_value());
  ASSERT_EQ(b.build(sample(t), t, kHostMinusPx4, out), ExternalVisionBuilder::Status::kOk);
  EXPECT_EQ(*b.lastAcceptedAge(t + 500 * kMs), 500 * kMs);
}

TEST(ExternalVision, LeverArmApplied)
{
  ExternalVisionBuilder::Config cfg;
  Pose lever;
  lever.translation = Eigen::Vector3d(0.1, 0.0, -0.05);  // PX4 body ref 10 cm fwd, 5 cm below base_link
  cfg.T_base_px4body = lever;
  ExternalVisionBuilder b(cfg);
  ExternalVisionSample out;
  const Nanoseconds t = kHostMinusPx4 + 5'000 * kMs;
  ASSERT_EQ(b.build(sample(t), t, kHostMinusPx4, out), ExternalVisionBuilder::Status::kOk);
  EXPECT_FLOAT_EQ(out.position[0], 1.1f);
  EXPECT_FLOAT_EQ(out.position[2], -2.95f);
}
