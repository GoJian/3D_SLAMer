// Builds the single map -> odom -> base_link chain from an estimator's pose stream (plan section 5).
//
// The estimator reports T_E_I: IMU pose in its own world frame E. That stream can jump on loop
// closure and restarts on estimator reset. Flight control needs continuous odometry, so:
//
//   T_E_B(k)      = T_E_I(k) * T_I_B                (T_I_B = inverse(T_base_imu) from calibration)
//   T_odom_B(k)   = T_odom_B(k-1) * [T_E_B(k-1)^-1 * T_E_B(k)]   for normal updates
//   T_odom_B(k)   = T_odom_B(k-1) * [v*dt dead-reckoned step]   for jump / reset updates
//   (v*dt uses the mean of the previous and current body-frame twist)
//   T_map_odom(k) = T_E_B(k) * T_odom_B(k)^-1       (E is used as `map`)
//
// So discrete corrections appear only in map -> odom; odom -> base_link never jumps.
// A jump is either flagged by the estimator, implied by a reset, or detected when the increment
// disagrees with the reported velocity by more than a tolerance that grows with the gap length.
// A short estimator dropout (dt <= max_gap) is NOT a jump by itself: the increment across it is
// real motion, and discarding it would offset odom from what the flight controller integrated
// with its IMU (SITL: 0.84 m after a 1 s dropout, docs/decisions/0003). Only gaps longer than
// max_gap are dead-reckoned unconditionally.
//
// After a reset the estimator's E is a new frame; T_map_odom is re-anchored so map stays defined
// by the new estimator world. Consumers must treat map as revisable (pose_revision increments).
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "x650_common/rigid.hpp"
#include "x650_common/time_mapping.hpp"

namespace x650_estimator_adapter
{

using x650_common::Nanoseconds;
using x650_common::Odometry;
using x650_common::Pose;

enum class TrackingState : std::uint8_t
{
  kUninitialized = 0,
  kInitializing = 1,
  kTracking = 2,
  kLost = 3,
};

struct EstimatorUpdate
{
  Nanoseconds stamp{0};             // acquisition time of the newest image used, host clock
  Odometry T_E_I;                   // IMU pose/twist/covariance in estimator world E
  std::uint64_t reset_count{0};     // estimator's own reset counter (monotonic)
  bool global_correction{false};    // estimator says this sample includes a loop-closure jump
  bool vision_updated{true};        // false = IMU-propagated only (plan section 7)
  TrackingState state{TrackingState::kTracking};
  float quality{1.0f};              // 0..1
};

struct SplitterOutput
{
  Nanoseconds stamp{0};
  Odometry odom_base;               // T_odom_base + base-frame twist; continuous
  Pose T_map_odom;
  bool jump_absorbed{false};        // this update moved map -> odom
  bool reset_absorbed{false};       // this update absorbed an estimator reset
  std::uint64_t pose_revision{0};   // increments whenever map -> odom changes discontinuously
  std::uint64_t reset_count{0};     // resets seen by the adapter
  bool vision_updated{true};
  TrackingState state{TrackingState::kTracking};
  float quality{1.0f};
};

class OdometrySplitter
{
public:
  struct Config
  {
    Pose T_base_imu;                   // from calibration: maps IMU coords into base_link
    double max_translation_residual_m{0.25};  // increment vs v*dt disagreement -> jump
    double max_rotation_residual_rad{0.20};   // increment vs w*dt disagreement -> jump
    double translation_residual_per_s{0.25};  // tolerance growth per second of gap
    double rotation_residual_per_s{0.20};
    Nanoseconds max_gap{2'000'000'000};       // longer gaps: dead-reckon, never trust the increment
  };

  explicit OdometrySplitter(const Config & config)
  : config_(config), T_I_B_(x650_common::inverse(config.T_base_imu)) {}

  // Returns nullopt for out-of-order or non-finite input (never emitted, never used as state).
  std::optional<SplitterOutput> update(const EstimatorUpdate & in)
  {
    if (!finite(in.T_E_I)) {
      return std::nullopt;
    }
    if (last_ && in.stamp <= last_->stamp) {
      return std::nullopt;
    }
    const Odometry E_B = x650_common::changeBodyFrame(in.T_E_I, T_I_B_);

    SplitterOutput out;
    out.stamp = in.stamp;
    out.vision_updated = in.vision_updated;
    out.state = in.state;
    out.quality = in.quality;

    if (!last_) {
      // First sample: odom coincides with the estimator world at start.
      odom_base_ = E_B;
      T_map_odom_ = Pose{};
    } else {
      const double dt = 1e-9 * static_cast<double>(in.stamp - last_->stamp);
      const bool reset = in.reset_count != last_reset_count_;
      const Pose inc = x650_common::compose(x650_common::inverse(last_E_B_.T_world_body), E_B.T_world_body);
      const bool jump = in.global_correction || reset ||
        in.stamp - last_->stamp > config_.max_gap || inconsistent(inc, E_B, dt);

      Pose step = jump ? deadReckon(last_E_B_, E_B, dt) : inc;
      odom_base_.T_world_body = x650_common::compose(odom_base_.T_world_body, step);
      // Body-frame twist is frame-independent of the world, copy it.
      odom_base_.linear_velocity_body = E_B.linear_velocity_body;
      odom_base_.angular_velocity_body = E_B.angular_velocity_body;
      odom_base_.velocity_covariance = E_B.velocity_covariance;
      // World-axis covariances: rotate from E into odom using the current odom/E alignment.
      const Eigen::Matrix3d R_odom_E =
        (odom_base_.T_world_body.rotation * E_B.T_world_body.rotation.inverse()).toRotationMatrix();
      odom_base_.position_covariance = x650_common::rotateCovariance(R_odom_E, E_B.position_covariance);
      odom_base_.orientation_covariance =
        x650_common::rotateCovariance(R_odom_E, E_B.orientation_covariance);

      if (jump) {
        ++pose_revision_;
        out.jump_absorbed = true;
      }
      if (reset) {
        ++reset_count_;
        out.reset_absorbed = true;
      }
      T_map_odom_ = x650_common::compose(E_B.T_world_body, x650_common::inverse(odom_base_.T_world_body));
    }

    last_E_B_ = E_B;
    last_reset_count_ = in.reset_count;
    out.odom_base = odom_base_;
    out.T_map_odom = T_map_odom_;
    out.pose_revision = pose_revision_;
    out.reset_count = reset_count_;
    last_ = out;
    return out;
  }

  const std::optional<SplitterOutput> & last() const {return last_;}

private:
  static bool finite(const Odometry & o)
  {
    return o.T_world_body.translation.allFinite() && o.T_world_body.rotation.coeffs().allFinite() &&
           o.T_world_body.rotation.norm() > 1e-6 && o.linear_velocity_body.allFinite() &&
           o.angular_velocity_body.allFinite();
  }

  // Constant-twist step over dt from the mean of the previous and current body-frame twist
  // (body twist is world-independent, so the current one is valid even across a jump/reset).
  static Pose deadReckon(const Odometry & prev, const Odometry & cur, double dt)
  {
    Pose step;
    step.translation = 0.5 * (prev.linear_velocity_body + cur.linear_velocity_body) * dt;
    const Eigen::Vector3d rot = 0.5 * (prev.angular_velocity_body + cur.angular_velocity_body) * dt;
    const double angle = rot.norm();
    step.rotation = angle > 0.0 ?
      Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot / angle)) : Eigen::Quaterniond::Identity();
    return step;
  }

  bool inconsistent(const Pose & inc, const Odometry & cur, double dt) const
  {
    const Pose predicted = deadReckon(last_E_B_, cur, dt);
    const double dtrans = (inc.translation - predicted.translation).norm();
    const double drot = inc.rotation.angularDistance(predicted.rotation);
    return dtrans > config_.max_translation_residual_m + config_.translation_residual_per_s * dt ||
           drot > config_.max_rotation_residual_rad + config_.rotation_residual_per_s * dt;
  }

  Config config_;
  Pose T_I_B_;
  std::optional<SplitterOutput> last_;
  Odometry last_E_B_;
  Odometry odom_base_;
  Pose T_map_odom_;
  std::uint64_t last_reset_count_{0};
  std::uint64_t pose_revision_{0};
  std::uint64_t reset_count_{0};
};

}  // namespace x650_estimator_adapter
