// Frame conversion between ROS (REP-103) and PX4 conventions.
//
// Notation: T_A_B maps coordinates from frame B into frame A. R_A_B is its rotation.
//
// ROS side (input):
//   world  = `odom`, z up (REP-103). Gravity-aligned; heading is whatever the estimator chose.
//   body   = `base_link`, FLU (x forward, y left, z up).
// PX4 side (output):
//   world  = NED (geographic, x north) or FRD (z down, arbitrary heading), see WorldConversion.
//   body   = FRD (x forward, y right, z down).
//
// A VIO `odom` frame is gravity-aligned but its heading is arbitrary, so the default is
// WorldConversion::kZUpToFrd (rotate 180 deg about x; PX4 POSE_FRAME_FRD). Use kEnuToNed only when
// the ROS world frame really is east-north-up (e.g. after a GNSS heading alignment); that one swaps
// x and y and flips z (PX4 POSE_FRAME_NED).
//
// Units: meters, seconds, radians. Quaternions are Hamilton, stored w-first in Eigen.
#pragma once

#include <Eigen/Geometry>

#include "x650_common/rigid.hpp"

namespace x650_px4_bridge
{

using x650_common::Odometry;
using x650_common::Pose;
using x650_common::changeBodyFrame;
using x650_common::rotateCovariance;
using x650_common::velocityInWorld;

enum class WorldConversion
{
  kZUpToFrd,  // x_frd = x, y_frd = -y, z_frd = -z  (default for VIO odom)
  kEnuToNed,  // x_ned = y_enu (north), y_ned = x_enu (east), z_ned = -z_enu
};

// R_frd_flu: maps FLU body coordinates into FRD body coordinates. Self-inverse.
inline Eigen::Matrix3d R_frd_flu()
{
  return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
}

// R_px4world_rosworld: maps ROS world coordinates into PX4 world coordinates. Self-inverse.
inline Eigen::Matrix3d R_px4world_rosworld(WorldConversion conv)
{
  Eigen::Matrix3d R;
  switch (conv) {
    case WorldConversion::kEnuToNed:
      R << 0.0, 1.0, 0.0,
           1.0, 0.0, 0.0,
           0.0, 0.0, -1.0;
      return R;
    case WorldConversion::kZUpToFrd:
    default:
      return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  }
}

// Convert a pose T_rosworld_flu into T_px4world_frd.
inline Pose rosToPx4Pose(const Pose & T_rosworld_flu, WorldConversion conv)
{
  const Eigen::Matrix3d R_w = R_px4world_rosworld(conv);
  const Eigen::Matrix3d R_flu_frd = R_frd_flu();  // self-inverse
  Pose out;
  out.translation = R_w * T_rosworld_flu.translation;
  const Eigen::Matrix3d R = R_w * T_rosworld_flu.rotation.normalized().toRotationMatrix() * R_flu_frd;
  out.rotation = Eigen::Quaterniond(R).normalized();
  return out;
}

// Inverse of rosToPx4Pose. Both fixed rotations are self-inverse, so the same formula applies.
inline Pose px4ToRosPose(const Pose & T_px4world_frd, WorldConversion conv)
{
  return rosToPx4Pose(T_px4world_frd, conv);
}

// Convert full odometry from ROS (world z-up, body FLU) to PX4 (world NED/FRD, body FRD).
inline Odometry rosToPx4(const Odometry & in, WorldConversion conv)
{
  const Eigen::Matrix3d R_w = R_px4world_rosworld(conv);
  const Eigen::Matrix3d R_b = R_frd_flu();
  Odometry out;
  out.T_world_body = rosToPx4Pose(in.T_world_body, conv);
  out.linear_velocity_body = R_b * in.linear_velocity_body;
  out.angular_velocity_body = R_b * in.angular_velocity_body;
  out.position_covariance = rotateCovariance(R_w, in.position_covariance);
  out.orientation_covariance = rotateCovariance(R_w, in.orientation_covariance);
  out.velocity_covariance = rotateCovariance(R_b, in.velocity_covariance);
  return out;
}

// Inverse of rosToPx4 (all fixed rotations are self-inverse).
inline Odometry px4ToRos(const Odometry & in, WorldConversion conv)
{
  return rosToPx4(in, conv);
}

}  // namespace x650_px4_bridge
