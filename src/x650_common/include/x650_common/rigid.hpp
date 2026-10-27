// Rigid-body types shared across x650 packages.
// Notation: T_A_B maps coordinates from frame B into frame A. Units: m, s, rad.
// Quaternions are Hamilton (Eigen, w-first constructor).
#pragma once

#include <Eigen/Geometry>

namespace x650_common
{

// Rigid pose T_A_B: p_A = rotation * p_B + translation.
struct Pose
{
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

  Eigen::Isometry3d isometry() const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = rotation.normalized().toRotationMatrix();
    T.translation() = translation;
    return T;
  }

  static Pose fromIsometry(const Eigen::Isometry3d & T)
  {
    Pose p;
    p.translation = T.translation();
    p.rotation = Eigen::Quaterniond(T.rotation()).normalized();
    return p;
  }
};

// Odometry in one convention. Velocities are expressed in the BODY frame (as in nav_msgs/Odometry,
// whose twist is in child_frame_id). Covariances are 3x3 blocks; rotational covariance is about
// the world-frame axes (small-angle perturbation applied on the left).
struct Odometry
{
  Pose T_world_body;
  Eigen::Vector3d linear_velocity_body{Eigen::Vector3d::Zero()};   // m/s
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};  // rad/s
  Eigen::Matrix3d position_covariance{Eigen::Matrix3d::Zero()};    // m^2, world axes
  Eigen::Matrix3d orientation_covariance{Eigen::Matrix3d::Zero()}; // rad^2, world axes
  Eigen::Matrix3d velocity_covariance{Eigen::Matrix3d::Zero()};    // (m/s)^2, body axes
};

// Rotate a covariance: C_A = R_A_B * C_B * R_A_B^T.
inline Eigen::Matrix3d rotateCovariance(const Eigen::Matrix3d & R_A_B, const Eigen::Matrix3d & C_B)
{
  return R_A_B * C_B * R_A_B.transpose();
}

// Linear velocity of the body expressed in world axes: v_world = R_world_body * v_body.
inline Eigen::Vector3d velocityInWorld(const Odometry & odom)
{
  return odom.T_world_body.rotation.normalized() * odom.linear_velocity_body;
}

// Move odometry from body frame B to a rigidly attached frame C (lever arm), e.g. from
// base_link to the flight-controller reference point. T_B_C is fixed.
//   T_W_C = T_W_B * T_B_C
//   w_C   = R_C_B * w_B
//   v_C   = R_C_B * (v_B + w_B x t_B_C)
// Covariances are rotated only; the lever-arm contribution to velocity covariance is not added.
inline Odometry changeBodyFrame(const Odometry & in, const Pose & T_B_C)
{
  const Eigen::Matrix3d R_C_B = T_B_C.rotation.normalized().toRotationMatrix().transpose();
  Odometry out = in;
  out.T_world_body = Pose::fromIsometry(in.T_world_body.isometry() * T_B_C.isometry());
  out.angular_velocity_body = R_C_B * in.angular_velocity_body;
  out.linear_velocity_body =
    R_C_B * (in.linear_velocity_body + in.angular_velocity_body.cross(T_B_C.translation));
  out.velocity_covariance = rotateCovariance(R_C_B, in.velocity_covariance);
  return out;
}

inline Pose inverse(const Pose & T_A_B)
{
  return Pose::fromIsometry(T_A_B.isometry().inverse());
}

// T_A_C = T_A_B * T_B_C
inline Pose compose(const Pose & T_A_B, const Pose & T_B_C)
{
  return Pose::fromIsometry(T_A_B.isometry() * T_B_C.isometry());
}

}  // namespace x650_common
