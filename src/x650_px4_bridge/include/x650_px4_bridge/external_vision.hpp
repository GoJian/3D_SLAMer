// Transport-agnostic external-vision sample builder (plan section 7).
//
// Takes local odometry in ROS conventions (odom z-up, base_link FLU) and produces the fields PX4
// external vision needs, in PX4 conventions. The transport (px4_msgs/VehicleOdometry over
// uXRCE-DDS, or MAVLink ODOMETRY) is chosen in M3 and only copies these fields.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "x650_common/time_mapping.hpp"
#include "x650_px4_bridge/frame_conversion.hpp"

namespace x650_px4_bridge
{

struct ExternalVisionSample
{
  std::int64_t timestamp_sample_us{0};   // acquisition time of the underlying image, PX4 clock
  WorldConversion world{WorldConversion::kZUpToFrd};  // selects POSE_FRAME_FRD vs POSE_FRAME_NED
  std::array<float, 3> position{};        // m, PX4 world frame
  std::array<float, 4> q{};               // w, x, y, z: rotation FRD body -> PX4 world
  std::array<float, 3> velocity{};        // m/s, FRD body frame (VELOCITY_FRAME_BODY_FRD)
  std::array<float, 3> angular_velocity{};  // rad/s, FRD body frame
  std::array<float, 3> position_variance{};
  std::array<float, 3> orientation_variance{};
  std::array<float, 3> velocity_variance{};
  std::uint8_t reset_counter{0};
  std::int8_t quality{0};                 // 0..100, PX4 semantics; -1 = invalid
};

// Input from the estimator adapter (/vio/odometry plus health).
struct LocalOdometryInput
{
  x650_common::Nanoseconds acquisition_stamp{0};  // header.stamp, host clock (never arrival time)
  Odometry odom;                                  // ROS conventions
  std::uint64_t estimator_reset_count{0};
  float quality{1.0f};                            // 0..1 from the adapter
};

class ExternalVisionBuilder
{
public:
  struct Config
  {
    WorldConversion world{WorldConversion::kZUpToFrd};
    x650_common::Nanoseconds max_age{100'000'000};   // reject older than 100 ms
    x650_common::Nanoseconds max_future{5'000'000};  // reject more than 5 ms in the future
    std::optional<Pose> T_base_px4body;              // lever arm to PX4 body reference, if any
  };

  enum class Status { kOk, kStale, kFuture, kNonMonotonic, kNonFinite, kNoTimesync };

  explicit ExternalVisionBuilder(const Config & config)
  : config_(config), gate_(config.max_age, config.max_future) {}

  // host_minus_px4: timesync offset from the transport; nullopt until synchronized.
  Status build(
    const LocalOdometryInput & in, x650_common::Nanoseconds now,
    std::optional<x650_common::Nanoseconds> host_minus_px4, ExternalVisionSample & out)
  {
    if (!isFinite(in.odom)) {
      return Status::kNonFinite;
    }
    if (!host_minus_px4) {
      return Status::kNoTimesync;
    }
    // An estimator reset may restart its time base; let the gate accept it and bump PX4's counter.
    if (has_reset_count_ && in.estimator_reset_count != last_reset_count_) {
      gate_.reset();
      ++reset_counter_;
    }
    has_reset_count_ = true;
    last_reset_count_ = in.estimator_reset_count;

    switch (gate_.check(in.acquisition_stamp, now)) {
      case x650_common::StalenessGate::Result::kStale: return Status::kStale;
      case x650_common::StalenessGate::Result::kFuture: return Status::kFuture;
      case x650_common::StalenessGate::Result::kNonMonotonic: return Status::kNonMonotonic;
      case x650_common::StalenessGate::Result::kOk: break;
    }

    Odometry ros = in.odom;
    if (config_.T_base_px4body) {
      ros = changeBodyFrame(ros, *config_.T_base_px4body);
    }
    const Odometry px4 = rosToPx4(ros, config_.world);

    out.timestamp_sample_us = x650_common::hostToPx4Micros(in.acquisition_stamp, *host_minus_px4);
    out.world = config_.world;
    copy3(px4.T_world_body.translation, out.position);
    const Eigen::Quaterniond q = px4.T_world_body.rotation.normalized();
    out.q = {static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
      static_cast<float>(q.z())};
    copy3(px4.linear_velocity_body, out.velocity);
    copy3(px4.angular_velocity_body, out.angular_velocity);
    copy3(px4.position_covariance.diagonal(), out.position_variance);
    copy3(px4.orientation_covariance.diagonal(), out.orientation_variance);
    copy3(px4.velocity_covariance.diagonal(), out.velocity_variance);
    out.reset_counter = reset_counter_;
    const float qn = std::fmin(std::fmax(in.quality, 0.0f), 1.0f);
    out.quality = static_cast<std::int8_t>(std::lround(qn * 100.0f));
    return Status::kOk;
  }

  // Age of the last accepted sample; the supervisor declares loss of vision when this grows.
  std::optional<x650_common::Nanoseconds> lastAcceptedAge(x650_common::Nanoseconds now) const
  {
    return gate_.lastAge(now);
  }

  std::uint8_t resetCounter() const {return reset_counter_;}

private:
  static void copy3(const Eigen::Vector3d & v, std::array<float, 3> & a)
  {
    for (int i = 0; i < 3; ++i) {
      a[i] = static_cast<float>(v[i]);
    }
  }

  static bool isFinite(const Odometry & o)
  {
    return o.T_world_body.translation.allFinite() && o.T_world_body.rotation.coeffs().allFinite() &&
           o.T_world_body.rotation.norm() > 1e-6 && o.linear_velocity_body.allFinite() &&
           o.angular_velocity_body.allFinite() && o.position_covariance.allFinite() &&
           o.orientation_covariance.allFinite() && o.velocity_covariance.allFinite();
  }

  Config config_;
  x650_common::StalenessGate gate_;
  bool has_reset_count_{false};
  std::uint64_t last_reset_count_{0};
  std::uint8_t reset_counter_{0};
};

inline const char * toString(ExternalVisionBuilder::Status s)
{
  switch (s) {
    case ExternalVisionBuilder::Status::kOk: return "ok";
    case ExternalVisionBuilder::Status::kStale: return "stale";
    case ExternalVisionBuilder::Status::kFuture: return "future";
    case ExternalVisionBuilder::Status::kNonMonotonic: return "non_monotonic";
    case ExternalVisionBuilder::Status::kNonFinite: return "non_finite";
    case ExternalVisionBuilder::Status::kNoTimesync: return "no_timesync";
  }
  return "unknown";
}

}  // namespace x650_px4_bridge
