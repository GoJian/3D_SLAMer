"""Fake VIO for PX4 SITL: Gazebo ground truth -> estimator-style odometry with injectable faults.

Stands in for OKVIS2-X until it runs here. Not evidence of estimator performance.

  in:  ~/input/groundtruth   nav_msgs/Odometry  (gz OdometryPublisher via ros_gz_bridge:
                                                 pose in the gz world (ENU), twist in body FLU)
       /sim/fault            std_msgs/String    "dropout <s>" | "jump <m>" | "reset" |
                                                "latency <s> [duration_s]"
  out: /estimator/odometry   nav_msgs/Odometry  pose of imu_link in `estimator_world`
       /estimator/reset_count std_msgs/UInt64
       /groundtruth/odometry nav_msgs/Odometry  gz world -> base_link, same stamps (evaluation)

Like a real VIO, the estimator world is gravity-aligned, starts at the first pose, and its heading
is offset from the vehicle's start heading by `yaw_offset_rad` (it does not know north).

Stamps (`stamp_source`): "message" uses the ground-truth header stamp (Gazebo sim time; use with
use_sim_time), "arrival" uses the node clock when the sample arrived (wall-clock mode). Either is
the stand-in "image acquisition" time. Output is published `latency_s` later on the node clock.
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass

import numpy as np


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def make_T(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


@dataclass
class FakeVioConfig:
    yaw_offset_rad: float = 0.6
    position_noise_m: float = 0.003       # white noise per sample
    drift_m_per_sqrt_s: float = 0.002     # random-walk drift
    latency_s: float = 0.025              # acquisition -> publish
    rate_hz: float = 30.0
    position_std_reported_m: float = 0.02
    orientation_std_reported_rad: float = 0.02
    velocity_std_reported_mps: float = 0.05
    seed: int = 0


@dataclass
class VioSample:
    stamp_ns: int
    T_E_I: np.ndarray                     # 4x4 IMU pose in estimator world
    v_body: np.ndarray
    w_body: np.ndarray
    reset_count: int


class FakeVio:
    """Pure state machine; the ROS node feeds it ground truth and publishes its output."""

    def __init__(self, cfg: FakeVioConfig):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.T_W_E: np.ndarray | None = None
        self.drift = np.zeros(3)
        self.jump_offset = np.zeros(3)
        self.reset_count = 0
        self.dropout_until_ns = -1
        self.latency_restore: tuple[int, float] | None = None   # (at_ns, latency_s)
        self.last_out_ns: int | None = None
        self.last_drift_ns: int | None = None
        self.events: list[tuple[int, str]] = []

    # Faults --------------------------------------------------------------------------------
    def command(self, text: str, now_ns: int) -> str:
        parts = text.split()
        if not parts:
            return 'empty command'
        kind = parts[0]
        if kind == 'dropout':
            d = float(parts[1]) if len(parts) > 1 else 1.0
            self.dropout_until_ns = now_ns + int(d * 1e9)
        elif kind == 'jump':
            d = float(parts[1]) if len(parts) > 1 else 0.5
            self.jump_offset = self.jump_offset + np.array([d, 0.0, 0.0])
        elif kind == 'reset':
            self.T_W_E = None          # re-anchor at the next sample
            self.drift[:] = 0.0
            self.jump_offset[:] = 0.0
            self.reset_count += 1
        elif kind == 'latency':
            if len(parts) > 2:
                self.latency_restore = (now_ns + int(float(parts[2]) * 1e9), self.cfg.latency_s)
            self.cfg.latency_s = float(parts[1])
        else:
            return f'unknown fault {kind!r}'
        self.events.append((now_ns, text))
        return 'ok'

    # Processing ----------------------------------------------------------------------------
    def process(self, stamp_ns: int, T_W_B: np.ndarray, v_body, w_body) -> VioSample | None:
        if self.latency_restore and stamp_ns >= self.latency_restore[0]:
            self.cfg.latency_s = self.latency_restore[1]
            self.latency_restore = None
        if self.last_out_ns is not None and stamp_ns - self.last_out_ns < 1e9 / self.cfg.rate_hz * 0.95:
            return None
        if stamp_ns < self.dropout_until_ns:
            return None
        if self.T_W_E is None:
            self.T_W_E = make_T(rot_z(yaw_of(T_W_B[:3, :3]) + self.cfg.yaw_offset_rad), T_W_B[:3, 3])
        if self.last_drift_ns is not None:
            dt = max(0.0, (stamp_ns - self.last_drift_ns) * 1e-9)
            self.drift += self.rng.normal(0.0, self.cfg.drift_m_per_sqrt_s * math.sqrt(dt), 3)
        self.last_drift_ns = stamp_ns
        T_E_I = inv_T(self.T_W_E) @ T_W_B          # T_base_imu = identity in simulation
        T_E_I[:3, 3] += self.drift + self.jump_offset + self.rng.normal(0.0, self.cfg.position_noise_m, 3)
        self.last_out_ns = stamp_ns
        return VioSample(stamp_ns, T_E_I, np.asarray(v_body, float), np.asarray(w_body, float),
                         self.reset_count)


class DelayLine:
    """Bounded FIFO releasing items once now >= stamp + latency."""

    def __init__(self, maxlen=256):
        self.q: collections.deque = collections.deque(maxlen=maxlen)

    def push(self, release_ns: int, item) -> None:
        self.q.append((release_ns, item))

    def pop_ready(self, now_ns: int) -> list:
        out = []
        while self.q and self.q[0][0] <= now_ns:
            out.append(self.q.popleft()[1])
        return out


# ROS node ----------------------------------------------------------------------------------
def main(argv=None) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String, UInt64

    from x650_evaluation.trajectory import make_pose, matrix_to_quat_xyzw

    class FakeVioNode(Node):
        def __init__(self):
            super().__init__('fake_vio')
            d = FakeVioConfig()
            cfg = FakeVioConfig(**{k: self.declare_parameter(k, getattr(d, k)).value
                                   for k in d.__dataclass_fields__})
            self.stamp_from_msg = self.declare_parameter('stamp_source', 'arrival').value == 'message'
            self.vio = FakeVio(cfg)
            self.delay = DelayLine()
            self.odom_pub = self.create_publisher(Odometry, '/estimator/odometry', 10)
            self.gt_pub = self.create_publisher(Odometry, '/groundtruth/odometry', 10)
            self.reset_pub = self.create_publisher(UInt64, '/estimator/reset_count',
                                                   QoSProfile(depth=10))
            self.create_subscription(Odometry, '~/input/groundtruth', self.on_gt, qos_profile_sensor_data)
            self.create_subscription(String, '/sim/fault', self.on_fault, 10)
            self.create_timer(0.002, self.flush)
            self.create_timer(1.0, self.publish_reset_count)
            self.make_pose, self.to_quat = make_pose, matrix_to_quat_xyzw
            self.published_reset_count = 0

        def now_ns(self) -> int:
            return self.get_clock().now().nanoseconds

        def on_fault(self, msg):
            r = self.vio.command(msg.data, self.now_ns())
            self.get_logger().warn(f'fault "{msg.data}": {r}')

        def publish_reset_count(self):
            # Count that matches the odometry already published, never ahead of it: samples
            # from the old estimator world may still be in the delay line after a reset.
            self.reset_pub.publish(UInt64(data=self.published_reset_count))

        def on_gt(self, m):
            t = (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec) if self.stamp_from_msg \
                else self.now_ns()
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = self.make_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            tw = m.twist.twist
            v = [tw.linear.x, tw.linear.y, tw.linear.z]
            w = [tw.angular.x, tw.angular.y, tw.angular.z]
            gt = Odometry()
            gt.header.stamp = rclpy.time.Time(nanoseconds=t).to_msg()
            gt.header.frame_id, gt.child_frame_id = 'gz_world', 'base_link'
            gt.pose, gt.twist = m.pose, m.twist
            self.gt_pub.publish(gt)
            s = self.vio.process(t, T, v, w)
            if s is not None:
                self.delay.push(t + int(self.vio.cfg.latency_s * 1e9), s)

        def flush(self):
            c = self.vio.cfg
            for s in self.delay.pop_ready(self.now_ns()):
                if s.reset_count != self.published_reset_count:
                    self.published_reset_count = s.reset_count
                    self.publish_reset_count()   # sent right before the first new-world sample
                o = Odometry()
                o.header.stamp = rclpy.time.Time(nanoseconds=s.stamp_ns).to_msg()
                o.header.frame_id, o.child_frame_id = 'estimator_world', 'imu_link'
                pp = o.pose.pose
                pp.position.x, pp.position.y, pp.position.z = map(float, s.T_E_I[:3, 3])
                qx, qy, qz, qw = map(float, self.to_quat(s.T_E_I[:3, :3]))
                pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
                o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, s.v_body)
                o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = map(float, s.w_body)
                for i in range(3):
                    o.pose.covariance[i * 7] = c.position_std_reported_m ** 2
                    o.pose.covariance[21 + i * 7] = c.orientation_std_reported_rad ** 2
                    o.twist.covariance[i * 7] = c.velocity_std_reported_mps ** 2
                self.odom_pub.publish(o)

    rclpy.init(args=argv)
    node = FakeVioNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
