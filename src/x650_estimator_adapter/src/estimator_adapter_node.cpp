// Estimator adapter node (M2).
//
// Subscribes to an estimator's nav_msgs/Odometry (pose of the IMU in the estimator world, twist in
// the IMU frame) and publishes the project contract:
//   /vio/odometry  nav_msgs/Odometry, frame odom -> base_link, continuous
//   /tf            map -> odom (discrete corrections) and odom -> base_link
//   /vio/health    diagnostic_msgs/DiagnosticArray
//
// Header stamps are passed through: the estimator must stamp with image acquisition time.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "x650_estimator_adapter/odometry_splitter.hpp"

namespace x650_estimator_adapter
{

namespace
{
Nanoseconds toNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<Nanoseconds>(t.sec) * 1'000'000'000LL + t.nanosec;
}

builtin_interfaces::msg::Time toMsg(Nanoseconds ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  return t;
}

Eigen::Matrix3d block(const std::array<double, 36> & c, int r0, int c0)
{
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) {
      m(r, k) = c[(r0 + r) * 6 + (c0 + k)];
    }
  }
  return m;
}

void setBlock(std::array<double, 36> & c, int r0, int c0, const Eigen::Matrix3d & m)
{
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) {
      c[(r0 + r) * 6 + (c0 + k)] = m(r, k);
    }
  }
}

geometry_msgs::msg::Transform toTransform(const Pose & p)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = p.translation.x();
  t.translation.y = p.translation.y();
  t.translation.z = p.translation.z();
  t.rotation.w = p.rotation.w();
  t.rotation.x = p.rotation.x();
  t.rotation.y = p.rotation.y();
  t.rotation.z = p.rotation.z();
  return t;
}

const char * stateName(TrackingState s)
{
  switch (s) {
    case TrackingState::kUninitialized: return "uninitialized";
    case TrackingState::kInitializing: return "initializing";
    case TrackingState::kTracking: return "tracking";
    case TrackingState::kLost: return "lost";
  }
  return "unknown";
}
}  // namespace

class EstimatorAdapterNode : public rclcpp::Node
{
public:
  explicit EstimatorAdapterNode(const rclcpp::NodeOptions & options)
  : Node("estimator_adapter", options), splitter_(makeConfig())
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    stale_after_ = static_cast<Nanoseconds>(1e9 * declare_parameter<double>("stale_after_s", 0.2));

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odometry", qos);
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/vio/health", qos);
    if (publish_tf_) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdometry(*msg);});
    reset_sub_ = create_subscription<std_msgs::msg::UInt64>(
      "~/input/reset_count", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {estimator_reset_count_ = msg->data;});
    health_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {publishHealth();});
  }

private:
  OdometrySplitter::Config makeConfig()
  {
    OdometrySplitter::Config c;
    const auto t = declare_parameter<std::vector<double>>("T_base_imu.translation", {0.0, 0.0, 0.0});
    const auto q = declare_parameter<std::vector<double>>("T_base_imu.rotation_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (t.size() != 3 || q.size() != 4) {
      throw std::invalid_argument("T_base_imu needs translation[3] and rotation_xyzw[4]");
    }
    c.T_base_imu.translation = Eigen::Vector3d(t[0], t[1], t[2]);
    c.T_base_imu.rotation = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
    c.max_translation_residual_m = declare_parameter<double>("jump.max_translation_residual_m", 0.25);
    c.max_rotation_residual_rad = declare_parameter<double>("jump.max_rotation_residual_rad", 0.20);
    c.translation_residual_per_s = declare_parameter<double>("jump.translation_residual_per_s", 0.25);
    c.rotation_residual_per_s = declare_parameter<double>("jump.rotation_residual_per_s", 0.20);
    c.max_gap = static_cast<Nanoseconds>(1e9 * declare_parameter<double>("jump.max_gap_s", 2.0));
    return c;
  }

  void onOdometry(const nav_msgs::msg::Odometry & msg)
  {
    EstimatorUpdate in;
    in.stamp = toNs(msg.header.stamp);
    const auto & p = msg.pose.pose;
    in.T_E_I.T_world_body.translation = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    in.T_E_I.T_world_body.rotation =
      Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    const auto & tw = msg.twist.twist;
    in.T_E_I.linear_velocity_body = Eigen::Vector3d(tw.linear.x, tw.linear.y, tw.linear.z);
    in.T_E_I.angular_velocity_body = Eigen::Vector3d(tw.angular.x, tw.angular.y, tw.angular.z);
    in.T_E_I.position_covariance = block(msg.pose.covariance, 0, 0);
    in.T_E_I.orientation_covariance = block(msg.pose.covariance, 3, 3);
    in.T_E_I.velocity_covariance = block(msg.twist.covariance, 0, 0);
    in.reset_count = estimator_reset_count_;

    const auto out = splitter_.update(in);
    if (!out) {
      ++rejected_;
      return;
    }
    ++accepted_;
    if (out->jump_absorbed) {
      ++jumps_;
    }

    nav_msgs::msg::Odometry o;
    o.header.stamp = toMsg(out->stamp);
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    const Pose & T = out->odom_base.T_world_body;
    o.pose.pose.position.x = T.translation.x();
    o.pose.pose.position.y = T.translation.y();
    o.pose.pose.position.z = T.translation.z();
    o.pose.pose.orientation.w = T.rotation.w();
    o.pose.pose.orientation.x = T.rotation.x();
    o.pose.pose.orientation.y = T.rotation.y();
    o.pose.pose.orientation.z = T.rotation.z();
    setBlock(o.pose.covariance, 0, 0, out->odom_base.position_covariance);
    setBlock(o.pose.covariance, 3, 3, out->odom_base.orientation_covariance);
    o.twist.twist.linear.x = out->odom_base.linear_velocity_body.x();
    o.twist.twist.linear.y = out->odom_base.linear_velocity_body.y();
    o.twist.twist.linear.z = out->odom_base.linear_velocity_body.z();
    o.twist.twist.angular.x = out->odom_base.angular_velocity_body.x();
    o.twist.twist.angular.y = out->odom_base.angular_velocity_body.y();
    o.twist.twist.angular.z = out->odom_base.angular_velocity_body.z();
    setBlock(o.twist.covariance, 0, 0, out->odom_base.velocity_covariance);
    odom_pub_->publish(o);

    if (tf_) {
      geometry_msgs::msg::TransformStamped a, b;
      a.header.stamp = b.header.stamp = o.header.stamp;
      a.header.frame_id = map_frame_;
      a.child_frame_id = odom_frame_;
      a.transform = toTransform(out->T_map_odom);
      b.header.frame_id = odom_frame_;
      b.child_frame_id = base_frame_;
      b.transform = toTransform(T);
      tf_->sendTransform({a, b});
    }
  }

  void publishHealth()
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "vio";
    s.hardware_id = "estimator_adapter";
    const auto & last = splitter_.last();
    const Nanoseconds now = get_clock()->now().nanoseconds();
    auto kv = [&s](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        s.values.push_back(e);
      };
    if (!last) {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "no estimate yet";
    } else {
      const Nanoseconds age = now - last->stamp;
      const bool stale = age > stale_after_;
      s.level = stale || last->state != TrackingState::kTracking ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = stale ? "stale" : stateName(last->state);
      kv("last_measurement_age_s", std::to_string(1e-9 * static_cast<double>(age)));
      kv("tracking_state", stateName(last->state));
      kv("pose_revision", std::to_string(last->pose_revision));
      kv("reset_count", std::to_string(last->reset_count));
      kv("quality", std::to_string(last->quality));
    }
    kv("accepted", std::to_string(accepted_));
    kv("rejected", std::to_string(rejected_));
    kv("jumps_absorbed", std::to_string(jumps_));
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = get_clock()->now();
    arr.status.push_back(s);
    health_pub_->publish(arr);
  }

  OdometrySplitter splitter_;
  std::string map_frame_, odom_frame_, base_frame_;
  bool publish_tf_{true};
  Nanoseconds stale_after_{200'000'000};
  std::uint64_t estimator_reset_count_{0};
  std::uint64_t accepted_{0}, rejected_{0}, jumps_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr reset_sub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace x650_estimator_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(x650_estimator_adapter::EstimatorAdapterNode)
