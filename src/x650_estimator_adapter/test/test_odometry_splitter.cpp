// Global pose corrections and estimator resets must not jump continuous local odometry.
#include <gtest/gtest.h>

#include "x650_estimator_adapter/odometry_splitter.hpp"

using namespace x650_estimator_adapter;

namespace
{
constexpr Nanoseconds kDt = 10'000'000;  // 100 Hz

// Straight-line motion at 1 m/s along estimator x; IMU == base.
EstimatorUpdate straight(int k, double x_offset = 0.0, std::uint64_t resets = 0)
{
  EstimatorUpdate u;
  u.stamp = 1'000'000'000 + k * kDt;
  u.T_E_I.T_world_body.translation = Eigen::Vector3d(0.01 * k + x_offset, 0, 0);
  u.T_E_I.linear_velocity_body = Eigen::Vector3d(1.0, 0, 0);
  u.reset_count = resets;
  return u;
}

OdometrySplitter makeSplitter()
{
  return OdometrySplitter(OdometrySplitter::Config{});
}

Eigen::Vector3d mapPosition(const SplitterOutput & o)
{
  return o.T_map_odom.isometry() * o.odom_base.T_world_body.translation;
}
}  // namespace

TEST(OdometrySplitter, NormalMotionPassesThrough)
{
  auto s = makeSplitter();
  for (int k = 0; k < 100; ++k) {
    const auto o = s.update(straight(k));
    ASSERT_TRUE(o);
    EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 0.01 * k, 1e-9);
    EXPECT_TRUE(o->T_map_odom.translation.isZero(1e-9));
    EXPECT_EQ(o->pose_revision, 0u);
  }
}

TEST(OdometrySplitter, FlaggedLoopClosureGoesIntoMapOdomOnly)
{
  auto s = makeSplitter();
  for (int k = 0; k < 50; ++k) {
    s.update(straight(k));
  }
  const double odom_before = s.last()->odom_base.T_world_body.translation.x();
  auto u = straight(50, /*x_offset=*/-0.4);  // loop closure pulls the estimate back 40 cm
  u.global_correction = true;
  const auto o = s.update(u);
  ASSERT_TRUE(o);
  EXPECT_TRUE(o->jump_absorbed);
  EXPECT_EQ(o->pose_revision, 1u);
  // odom advanced by v*dt only, no 40 cm jump.
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x() - odom_before, 0.01, 1e-9);
  // map-frame pose equals the corrected estimator pose.
  EXPECT_NEAR(mapPosition(*o).x(), 0.5 - 0.4, 1e-9);
  // Subsequent motion continues smoothly in odom; map->odom stays fixed.
  const auto o2 = s.update(straight(51, -0.4));
  EXPECT_FALSE(o2->jump_absorbed);
  EXPECT_NEAR(o2->odom_base.T_world_body.translation.x() - o->odom_base.T_world_body.translation.x(), 0.01, 1e-9);
  EXPECT_TRUE(o2->T_map_odom.translation.isApprox(o->T_map_odom.translation, 1e-9));
}

TEST(OdometrySplitter, UnflaggedJumpDetectedByResidual)
{
  auto s = makeSplitter();
  for (int k = 0; k < 20; ++k) {
    s.update(straight(k));
  }
  const auto o = s.update(straight(20, 1.0));
  ASSERT_TRUE(o);
  EXPECT_TRUE(o->jump_absorbed);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 0.20, 1e-9);
  EXPECT_NEAR(mapPosition(*o).x(), 1.20, 1e-9);
}

TEST(OdometrySplitter, RotationJumpDetected)
{
  auto s = makeSplitter();
  for (int k = 0; k < 20; ++k) {
    s.update(straight(k));
  }
  auto u = straight(20);
  u.T_E_I.T_world_body.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  const auto o = s.update(u);
  EXPECT_TRUE(o->jump_absorbed);
  EXPECT_LT(o->odom_base.T_world_body.rotation.angularDistance(Eigen::Quaterniond::Identity()), 1e-9);
  const Eigen::Quaterniond map_q = Eigen::Quaterniond(o->T_map_odom.rotation) * o->odom_base.T_world_body.rotation;
  EXPECT_NEAR(map_q.angularDistance(Eigen::Quaterniond::Identity()), 0.5, 1e-9);
}

TEST(OdometrySplitter, EstimatorResetKeepsOdomContinuous)
{
  auto s = makeSplitter();
  for (int k = 0; k < 30; ++k) {
    s.update(straight(k));
  }
  const double before = s.last()->odom_base.T_world_body.translation.x();
  // Reset: estimator world restarts at the origin.
  EstimatorUpdate u;
  u.stamp = 1'000'000'000 + 30 * kDt;
  u.T_E_I.linear_velocity_body = Eigen::Vector3d(1.0, 0, 0);
  u.reset_count = 1;
  const auto o = s.update(u);
  ASSERT_TRUE(o);
  EXPECT_TRUE(o->reset_absorbed);
  EXPECT_EQ(o->reset_count, 1u);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x() - before, 0.01, 1e-9);
  // map is now the new estimator world: base sits at its origin.
  EXPECT_TRUE(mapPosition(*o).isZero(1e-9));
}

TEST(OdometrySplitter, RejectsOutOfOrderAndNan)
{
  auto s = makeSplitter();
  ASSERT_TRUE(s.update(straight(5)));
  EXPECT_FALSE(s.update(straight(4)));
  EXPECT_FALSE(s.update(straight(5)));
  auto bad = straight(6);
  bad.T_E_I.T_world_body.translation.y() = std::nan("");
  EXPECT_FALSE(s.update(bad));
  EXPECT_TRUE(s.update(straight(6)));
}

TEST(OdometrySplitter, ImuToBaseExtrinsicApplied)
{
  OdometrySplitter::Config c;
  c.T_base_imu.translation = Eigen::Vector3d(0.1, 0.0, 0.05);  // IMU 10 cm forward, 5 cm up
  OdometrySplitter s(c);
  EstimatorUpdate u;
  u.stamp = 1;
  const auto o = s.update(u);  // IMU at E origin, so base is at -T_base_imu
  EXPECT_TRUE(o->odom_base.T_world_body.translation.isApprox(Eigen::Vector3d(-0.1, 0, -0.05), 1e-12));
}

TEST(OdometrySplitter, ShortDropoutKeepsRealMotion)
{
  auto s = makeSplitter();
  for (int k = 0; k < 10; ++k) {
    s.update(straight(k));
  }
  // 1 s estimator dropout (<= max_gap) with consistent motion: the increment is trusted.
  const auto o = s.update(straight(110));
  EXPECT_FALSE(o->jump_absorbed);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 1.10, 1e-9);
  EXPECT_TRUE(o->T_map_odom.translation.isZero(1e-9));
}

TEST(OdometrySplitter, DropoutWhileAcceleratingIsNotAJump)
{
  // SITL regression: hovering, 1 s dropout, vehicle starts the next leg during the gap.
  auto s = makeSplitter();
  EstimatorUpdate u;
  for (int k = 0; k < 10; ++k) {
    u.stamp = 1'000'000'000 + k * kDt;
    s.update(u);  // hover at origin, zero velocity
  }
  u.stamp += 1'000'000'000;
  u.T_E_I.T_world_body.translation = Eigen::Vector3d(0.75, 0, 0);
  u.T_E_I.linear_velocity_body = Eigen::Vector3d(1.5, 0, 0);
  const auto o = s.update(u);
  EXPECT_FALSE(o->jump_absorbed);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 0.75, 1e-9);
}

TEST(OdometrySplitter, GapBeyondMaxGapIsJumpBoundary)
{
  auto s = makeSplitter();
  for (int k = 0; k < 10; ++k) {
    s.update(straight(k));
  }
  // 3 s gap (> max_gap 2 s): never trust the increment; dead-reckon and push the rest to map.
  const auto o = s.update(straight(310));
  EXPECT_TRUE(o->jump_absorbed);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 0.09 + 3.01, 1e-9);
  EXPECT_NEAR(mapPosition(*o).x(), 3.10, 1e-9);
}

TEST(OdometrySplitter, LoopClosureAfterShortDropoutStillDetected)
{
  auto s = makeSplitter();
  for (int k = 0; k < 10; ++k) {
    s.update(straight(k));
  }
  // 0.5 s gap, consistent motion plus a 0.6 m correction: 0.6 > 0.25 + 0.25 * 0.5.
  const auto o = s.update(straight(59, 0.6));
  EXPECT_TRUE(o->jump_absorbed);
  EXPECT_NEAR(mapPosition(*o).x(), 0.59 + 0.6, 1e-9);
  EXPECT_NEAR(o->odom_base.T_world_body.translation.x(), 0.59, 1e-9);
}
