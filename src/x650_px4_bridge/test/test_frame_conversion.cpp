// Sprint item 5: transform round trips and known-axis motions, before any PX4 connection.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "x650_px4_bridge/frame_conversion.hpp"

using namespace x650_px4_bridge;

namespace
{
constexpr double kTol = 1e-12;
const WorldConversion kBoth[] = {WorldConversion::kZUpToFrd, WorldConversion::kEnuToNed};

Pose randomPose(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-10.0, 10.0);
  Pose p;
  p.translation = Eigen::Vector3d(u(rng), u(rng), u(rng));
  p.rotation = Eigen::Quaterniond(u(rng), u(rng), u(rng), u(rng)).normalized();
  return p;
}

Odometry yawed(double yaw)
{
  Odometry o;
  o.T_world_body.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return o;
}

double angleBetween(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  return a.angularDistance(b);
}
}  // namespace

TEST(FixedRotations, ProperAndSelfInverse)
{
  for (auto conv : kBoth) {
    const Eigen::Matrix3d R = R_px4world_rosworld(conv);
    EXPECT_NEAR(R.determinant(), 1.0, kTol);
    EXPECT_TRUE((R * R).isApprox(Eigen::Matrix3d::Identity(), kTol));
  }
  EXPECT_NEAR(R_frd_flu().determinant(), 1.0, kTol);
}

TEST(RoundTrip, PoseIsRecovered)
{
  std::mt19937 rng(7);
  for (auto conv : kBoth) {
    for (int i = 0; i < 1000; ++i) {
      const Pose p = randomPose(rng);
      const Pose back = px4ToRosPose(rosToPx4Pose(p, conv), conv);
      EXPECT_TRUE(back.translation.isApprox(p.translation, 1e-12));
      EXPECT_LT(angleBetween(back.rotation, p.rotation), 1e-9);
    }
  }
}

TEST(RoundTrip, OdometryIsRecovered)
{
  std::mt19937 rng(8);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  for (auto conv : kBoth) {
    Odometry o;
    o.T_world_body = randomPose(rng);
    o.linear_velocity_body = Eigen::Vector3d(u(rng), u(rng), u(rng));
    o.angular_velocity_body = Eigen::Vector3d(u(rng), u(rng), u(rng));
    Eigen::Matrix3d A = Eigen::Matrix3d::Random();
    o.position_covariance = A * A.transpose();
    o.orientation_covariance = 0.1 * A * A.transpose();
    o.velocity_covariance = 0.5 * A.transpose() * A;
    const Odometry back = px4ToRos(rosToPx4(o, conv), conv);
    EXPECT_TRUE(back.linear_velocity_body.isApprox(o.linear_velocity_body, kTol));
    EXPECT_TRUE(back.angular_velocity_body.isApprox(o.angular_velocity_body, kTol));
    EXPECT_TRUE(back.position_covariance.isApprox(o.position_covariance, kTol));
    EXPECT_TRUE(back.orientation_covariance.isApprox(o.orientation_covariance, kTol));
    EXPECT_TRUE(back.velocity_covariance.isApprox(o.velocity_covariance, kTol));
  }
}

// Identity attitude in ROS (nose along world x, level) is identity in PX4 FRD-world.
TEST(KnownAxis, LevelIdentityMapsToIdentityInFrd)
{
  const Pose px4 = rosToPx4Pose(Pose{}, WorldConversion::kZUpToFrd);
  EXPECT_LT(angleBetween(px4.rotation, Eigen::Quaterniond::Identity()), 1e-12);
}

// ENU identity: nose points east. In NED that is yaw = +90 deg (clockwise from north, viewed from above).
TEST(KnownAxis, EnuNoseEastIsNedYawPlus90)
{
  const Pose px4 = rosToPx4Pose(Pose{}, WorldConversion::kEnuToNed);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  EXPECT_LT(angleBetween(px4.rotation, expected), 1e-12);
}

// ENU yaw +90 deg (nose north) is NED yaw 0.
TEST(KnownAxis, EnuNoseNorthIsNedYawZero)
{
  const Pose px4 = rosToPx4Pose(yawed(M_PI / 2).T_world_body, WorldConversion::kEnuToNed);
  EXPECT_LT(angleBetween(px4.rotation, Eigen::Quaterniond::Identity()), 1e-12);
}

// A left turn (CCW from above, positive ROS yaw) is a negative PX4 yaw.
TEST(KnownAxis, LeftYawIsNegativeFrdYaw)
{
  const Pose px4 = rosToPx4Pose(yawed(0.3).T_world_body, WorldConversion::kZUpToFrd);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()));
  EXPECT_LT(angleBetween(px4.rotation, expected), 1e-12);
}

TEST(KnownAxis, ClimbIsNegativeZ)
{
  for (auto conv : kBoth) {
    Pose p;
    p.translation = Eigen::Vector3d(0, 0, 2.0);  // 2 m up
    EXPECT_NEAR(rosToPx4Pose(p, conv).translation.z(), -2.0, kTol);
  }
}

TEST(KnownAxis, MoveLeftIsNegativeYInFrd)
{
  Pose p;
  p.translation = Eigen::Vector3d(1.0, 0.5, 0.0);  // forward 1 m, left 0.5 m
  const Pose px4 = rosToPx4Pose(p, WorldConversion::kZUpToFrd);
  EXPECT_TRUE(px4.translation.isApprox(Eigen::Vector3d(1.0, -0.5, 0.0), kTol));
}

TEST(KnownAxis, EnuEastNorthMapToNedNorthEast)
{
  Pose p;
  p.translation = Eigen::Vector3d(3.0, 4.0, 1.0);  // 3 m east, 4 m north, 1 m up
  const Pose px4 = rosToPx4Pose(p, WorldConversion::kEnuToNed);
  EXPECT_TRUE(px4.translation.isApprox(Eigen::Vector3d(4.0, 3.0, -1.0), kTol));
}

TEST(KnownAxis, BodyRatesAndVelocity)
{
  Odometry o;
  o.linear_velocity_body = Eigen::Vector3d(2.0, 1.0, 0.5);    // fwd, left, up
  o.angular_velocity_body = Eigen::Vector3d(0.1, 0.2, 0.3);   // roll right?, pitch down?, yaw left
  const Odometry px4 = rosToPx4(o, WorldConversion::kZUpToFrd);
  EXPECT_TRUE(px4.linear_velocity_body.isApprox(Eigen::Vector3d(2.0, -1.0, -0.5), kTol));
  // FLU yaw-left rate +0.3 is FRD yaw rate -0.3 (FRD yaw positive = clockwise from above).
  EXPECT_TRUE(px4.angular_velocity_body.isApprox(Eigen::Vector3d(0.1, -0.2, -0.3), kTol));
}

// World-frame velocity is consistent across conventions: v_px4world = R * v_rosworld.
TEST(Consistency, WorldVelocityCommutes)
{
  std::mt19937 rng(9);
  for (auto conv : kBoth) {
    Odometry o;
    o.T_world_body = randomPose(rng);
    o.linear_velocity_body = Eigen::Vector3d(1.0, -2.0, 0.3);
    const Eigen::Vector3d v_ros_world = velocityInWorld(o);
    const Eigen::Vector3d v_px4_world = velocityInWorld(rosToPx4(o, conv));
    EXPECT_TRUE(v_px4_world.isApprox(R_px4world_rosworld(conv) * v_ros_world, 1e-12));
  }
}

// Points transform consistently: p_px4world = T_px4world_frd * (R_frd_flu * p_flu).
TEST(Consistency, BodyPointMapsToSameWorldPoint)
{
  std::mt19937 rng(10);
  for (auto conv : kBoth) {
    const Pose p = randomPose(rng);
    const Eigen::Vector3d point_flu(0.3, -0.2, 0.1);
    const Eigen::Vector3d world_ros = p.isometry() * point_flu;
    const Eigen::Vector3d world_px4 = rosToPx4Pose(p, conv).isometry() * (R_frd_flu() * point_flu);
    EXPECT_TRUE(world_px4.isApprox(R_px4world_rosworld(conv) * world_ros, 1e-12));
  }
}

TEST(LeverArm, PureTranslationAddsRotationalVelocity)
{
  Odometry o;
  o.angular_velocity_body = Eigen::Vector3d(0, 0, 1.0);  // 1 rad/s yaw left
  Pose T_B_C;
  T_B_C.translation = Eigen::Vector3d(0.5, 0, 0);        // C is 0.5 m forward
  const Odometry c = changeBodyFrame(o, T_B_C);
  EXPECT_TRUE(c.T_world_body.translation.isApprox(Eigen::Vector3d(0.5, 0, 0), kTol));
  // Forward point moves left at 0.5 m/s.
  EXPECT_TRUE(c.linear_velocity_body.isApprox(Eigen::Vector3d(0, 0.5, 0), kTol));
}

TEST(LeverArm, RoundTripThroughInverse)
{
  std::mt19937 rng(11);
  Odometry o;
  o.T_world_body = randomPose(rng);
  o.linear_velocity_body = Eigen::Vector3d(1, 2, 3);
  o.angular_velocity_body = Eigen::Vector3d(0.3, -0.2, 0.5);
  const Pose T_B_C = randomPose(rng);
  const Pose T_C_B = Pose::fromIsometry(T_B_C.isometry().inverse());
  const Odometry back = changeBodyFrame(changeBodyFrame(o, T_B_C), T_C_B);
  EXPECT_TRUE(back.T_world_body.translation.isApprox(o.T_world_body.translation, 1e-9));
  EXPECT_TRUE(back.linear_velocity_body.isApprox(o.linear_velocity_body, 1e-9));
  EXPECT_TRUE(back.angular_velocity_body.isApprox(o.angular_velocity_body, 1e-9));
}
