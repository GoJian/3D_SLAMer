// PX4 external-vision transport over uXRCE-DDS (M3, decision 0004).
//
//   in:  /vio/odometry                  nav_msgs/Odometry (odom -> base_link, REP-103, continuous)
//        /vio/health                    diagnostic_msgs/DiagnosticArray (quality, tracking state)
//        /fmu/out/timesync_status       px4_msgs/TimesyncStatus (link alive + offset, info only)
//        /fmu/out/vehicle_odometry      px4_msgs/VehicleOdometry (PX4 EKF output)
//   out: /fmu/in/vehicle_visual_odometry  px4_msgs/VehicleOdometry (PX4 conventions)
//        /px4/odometry                  nav_msgs/Odometry: PX4 EKF output converted back to REP-103
//        /px4_bridge/status             diagnostic_msgs/DiagnosticArray
//
// Timing: header.stamp of /vio/odometry is image acquisition time on the host clock. It is sent as
// timestamp_sample in host-clock microseconds; the PX4 uXRCE-DDS client converts timestamp and
// timestamp_sample from agent time to PX4 time itself (UXRCE_DDS_SYNCT=1), so no offset is applied
// here. EKF2_EV_DELAY should then be 0.
//
// Simulation (decision 0004): with UXRCE_DDS_SYNCT=0 and ROS on Gazebo's /clock, PX4 time equals
// ROS time and no conversion happens; set require_timesync:=false there.
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "x650_px4_bridge/external_vision.hpp"

namespace x650_px4_bridge
{
namespace
{
using x650_common::Nanoseconds;

Nanoseconds toNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<Nanoseconds>(t.sec) * 1'000'000'000LL + t.nanosec;
}

builtin_interfaces::msg::Time fromNs(Nanoseconds ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  return t;
}

Eigen::Matrix3d block(const std::array<double, 36> & c, int r0)
{
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) {
      m(r, k) = c[(r0 + r) * 6 + (r0 + k)];
    }
  }
  return m;
}

void setDiag(std::array<double, 36> & c, int r0, const std::array<float, 3> & v)
{
  for (int i = 0; i < 3; ++i) {
    c[(r0 + i) * 6 + (r0 + i)] = std::isfinite(v[i]) ? v[i] : -1.0;
  }
}

WorldConversion parseWorld(const std::string & s)
{
  if (s == "zup_to_frd") {
    return WorldConversion::kZUpToFrd;
  }
  if (s == "enu_to_ned") {
    return WorldConversion::kEnuToNed;
  }
  throw std::invalid_argument("world_conversion must be zup_to_frd or enu_to_ned, got " + s);
}
}  // namespace

class Px4BridgeNode : public rclcpp::Node
{
public:
  explicit Px4BridgeNode(const rclcpp::NodeOptions & options)
  : Node("px4_bridge", options), builder_(makeConfig())
  {
    timesync_timeout_ =
      static_cast<Nanoseconds>(1e9 * declare_parameter<double>("timesync_timeout_s", 2.0));
    min_quality_ = declare_parameter<double>("min_quality", 0.0);
    require_timesync_ = declare_parameter<bool>("require_timesync", true);
    const std::string ns = declare_parameter<std::string>("fmu_namespace", "/fmu");

    ev_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      ns + "/in/vehicle_visual_odometry", rclcpp::QoS(rclcpp::KeepLast(10)));
    px4_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/px4/odometry", rclcpp::QoS(10));
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/px4_bridge/status", rclcpp::QoS(10));

    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vio/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onVio(*m);});
    health_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/vio/health", rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {onHealth(*m);});
    timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      ns + "/out/timesync_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr m) {
        last_timesync_ = now().nanoseconds();
        px4_offset_us_ = m->estimated_offset;
      });
    px4_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      ns + "/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr m) {onPx4Odometry(*m);});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {publishStatus();});
  }

private:
  ExternalVisionBuilder::Config makeConfig()
  {
    ExternalVisionBuilder::Config c;
    world_ = parseWorld(declare_parameter<std::string>("world_conversion", "zup_to_frd"));
    c.world = world_;
    c.max_age = static_cast<Nanoseconds>(1e9 * declare_parameter<double>("max_age_s", 0.1));
    c.max_future = static_cast<Nanoseconds>(1e9 * declare_parameter<double>("max_future_s", 0.005));
    const auto t = declare_parameter<std::vector<double>>("T_base_px4body.translation", {0, 0, 0});
    const auto q =
      declare_parameter<std::vector<double>>("T_base_px4body.rotation_xyzw", {0, 0, 0, 1});
    if (t.size() != 3 || q.size() != 4) {
      throw std::invalid_argument("T_base_px4body needs translation[3] and rotation_xyzw[4]");
    }
    Pose p;
    p.translation = Eigen::Vector3d(t[0], t[1], t[2]);
    p.rotation = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
    if (!p.translation.isZero() || p.rotation.angularDistance(Eigen::Quaterniond::Identity()) > 0) {
      c.T_base_px4body = p;
    }
    return c;
  }

  bool timesyncOk(Nanoseconds now_ns) const
  {
    if (!require_timesync_) {
      return true;
    }
    return last_timesync_ && now_ns - *last_timesync_ < timesync_timeout_;
  }

  void onHealth(const diagnostic_msgs::msg::DiagnosticArray & m)
  {
    for (const auto & s : m.status) {
      if (s.name != "vio") {
        continue;
      }
      for (const auto & kv : s.values) {
        if (kv.key == "quality") {
          quality_ = std::stof(kv.value);
        }
      }
    }
  }

  void onVio(const nav_msgs::msg::Odometry & m)
  {
    const Nanoseconds now_ns = now().nanoseconds();
    LocalOdometryInput in;
    in.acquisition_stamp = toNs(m.header.stamp);
    const auto & p = m.pose.pose;
    in.odom.T_world_body.translation = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    in.odom.T_world_body.rotation =
      Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    const auto & tw = m.twist.twist;
    in.odom.linear_velocity_body = Eigen::Vector3d(tw.linear.x, tw.linear.y, tw.linear.z);
    in.odom.angular_velocity_body = Eigen::Vector3d(tw.angular.x, tw.angular.y, tw.angular.z);
    in.odom.position_covariance = block(m.pose.covariance, 0);
    in.odom.orientation_covariance = block(m.pose.covariance, 3);
    in.odom.velocity_covariance = block(m.twist.covariance, 0);
    in.quality = quality_;
    // /vio/odometry is continuous by contract (decision 0003): estimator resets are absorbed
    // upstream, so PX4's reset_counter only changes if that contract is broken.
    in.estimator_reset_count = 0;

    ExternalVisionSample s;
    const auto status = timesyncOk(now_ns) ?
      builder_.build(in, now_ns, Nanoseconds{0}, s) :
      builder_.build(in, now_ns, std::nullopt, s);
    ++counts_[toString(status)];
    if (status != ExternalVisionBuilder::Status::kOk) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "EV sample rejected: %s",
        toString(status));
      return;
    }
    if (quality_ < min_quality_) {
      ++counts_["low_quality"];
      return;
    }

    px4_msgs::msg::VehicleOdometry o;
    o.timestamp = static_cast<uint64_t>(now_ns / 1000);
    o.timestamp_sample = static_cast<uint64_t>(s.timestamp_sample_us);
    o.pose_frame = s.world == WorldConversion::kEnuToNed ?
      px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED : px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    o.position = s.position;
    o.q = s.q;
    o.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    o.velocity = s.velocity;
    o.angular_velocity = s.angular_velocity;
    o.position_variance = s.position_variance;
    o.orientation_variance = s.orientation_variance;
    o.velocity_variance = s.velocity_variance;
    o.reset_counter = s.reset_counter;
    o.quality = s.quality;
    ev_pub_->publish(o);
    last_sent_ = now_ns;
  }

  // PX4 EKF output -> REP-103 (frame px4_local: z-up world matching the EV convention).
  void onPx4Odometry(const px4_msgs::msg::VehicleOdometry & m)
  {
    if (m.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED &&
      m.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD &&
      m.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD)
    {
      return;
    }
    Odometry px4;
    px4.T_world_body.translation = Eigen::Vector3d(m.position[0], m.position[1], m.position[2]);
    px4.T_world_body.rotation = Eigen::Quaterniond(m.q[0], m.q[1], m.q[2], m.q[3]).normalized();
    const Eigen::Vector3d v(m.velocity[0], m.velocity[1], m.velocity[2]);
    px4.linear_velocity_body =
      m.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD ?
      v : Eigen::Vector3d(px4.T_world_body.rotation.inverse() * v);
    px4.angular_velocity_body =
      Eigen::Vector3d(m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]);
    px4.position_covariance.setZero();
    px4.orientation_covariance.setZero();
    px4.velocity_covariance.setZero();
    if (!px4.T_world_body.translation.allFinite() || !px4.T_world_body.rotation.coeffs().allFinite()) {
      return;
    }
    const Odometry ros = px4ToRos(px4, world_);

    nav_msgs::msg::Odometry o;
    // PX4 timestamps arrive already converted to the agent (host) clock.
    o.header.stamp = fromNs(static_cast<Nanoseconds>(m.timestamp_sample) * 1000);
    o.header.frame_id = "px4_local";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = ros.T_world_body.translation.x();
    o.pose.pose.position.y = ros.T_world_body.translation.y();
    o.pose.pose.position.z = ros.T_world_body.translation.z();
    o.pose.pose.orientation.w = ros.T_world_body.rotation.w();
    o.pose.pose.orientation.x = ros.T_world_body.rotation.x();
    o.pose.pose.orientation.y = ros.T_world_body.rotation.y();
    o.pose.pose.orientation.z = ros.T_world_body.rotation.z();
    o.twist.twist.linear.x = ros.linear_velocity_body.x();
    o.twist.twist.linear.y = ros.linear_velocity_body.y();
    o.twist.twist.linear.z = ros.linear_velocity_body.z();
    o.twist.twist.angular.x = ros.angular_velocity_body.x();
    o.twist.twist.angular.y = ros.angular_velocity_body.y();
    o.twist.twist.angular.z = ros.angular_velocity_body.z();
    // Variances are per-axis in PX4 frames; axis permutation keeps magnitudes (sign-free).
    std::array<float, 3> pv{m.position_variance[0], m.position_variance[1], m.position_variance[2]};
    if (world_ == WorldConversion::kEnuToNed) {
      std::swap(pv[0], pv[1]);
    }
    setDiag(o.pose.covariance, 0, pv);
    setDiag(o.pose.covariance, 3, m.orientation_variance);
    setDiag(o.twist.covariance, 0, m.velocity_variance);
    px4_odom_pub_->publish(o);
  }

  void publishStatus()
  {
    const Nanoseconds now_ns = now().nanoseconds();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "px4_bridge";
    s.hardware_id = "uxrce_dds";
    auto kv = [&s](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        s.values.push_back(e);
      };
    const bool sync = timesyncOk(now_ns);
    const auto age = builder_.lastAcceptedAge(now_ns);
    const bool sending = last_sent_ && now_ns - *last_sent_ < 200'000'000;
    s.level = sync && sending ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    s.message = !sync ? "no PX4 timesync" : (!sending ? "not sending" : "ok");
    kv("timesync", sync ? "true" : "false");
    kv("px4_estimated_offset_us", std::to_string(px4_offset_us_));
    kv("last_accepted_age_s", age ? std::to_string(1e-9 * static_cast<double>(*age)) : "none");
    kv("reset_counter", std::to_string(builder_.resetCounter()));
    for (const auto & [k, v] : counts_) {
      kv("count_" + k, std::to_string(v));
    }
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(s);
    status_pub_->publish(arr);
  }

  WorldConversion world_{WorldConversion::kZUpToFrd};
  ExternalVisionBuilder builder_;
  Nanoseconds timesync_timeout_{2'000'000'000};
  double min_quality_{0.0};
  bool require_timesync_{true};
  float quality_{1.0f};
  std::optional<Nanoseconds> last_timesync_, last_sent_;
  std::int64_t px4_offset_us_{0};
  std::map<std::string, std::uint64_t> counts_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace x650_px4_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(x650_px4_bridge::Px4BridgeNode)
