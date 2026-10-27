"""Synthetic sessions with known ground truth and injected faults.

Used to test the recording/audit/evaluation tooling before hardware exists, and for fault
injection (plan section 12: missing/late stereo frames, IMU gaps, estimator resets, global pose
corrections). Synthetic sessions are NOT evidence of system performance.

Topics written (project contract, plan section 6):
  /rig/cam0/image_raw, /rig/cam1/image_raw      sensor_msgs/Image (tiny mono8)
  /rig/cam{0,1}/frame_metadata                  x650_msgs/FrameMetadata
  /rig/imu                                      sensor_msgs/Imu (consistent with trajectory)
  /groundtruth/odometry                         nav_msgs/Odometry (map -> base_link)
  /estimator/odometry                           nav_msgs/Odometry (noisy, optional jump/reset)
  /estimator/reset_count                        std_msgs/UInt64

  x650_synthetic_session data/synthetic/loop --motion loop --duration 20 \
      --drop-frames 100,101 --imu-gap 5.0:0.05 --loop-closure 12.0:0.3
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from x650_evaluation.bag import BagWriter
from x650_evaluation.trajectory import Trajectory, matrix_to_quat_xyzw

GRAVITY = 9.80665
T0_NS = 1_700_000_000_000_000_000  # arbitrary epoch so stamps look like ROS wall time


@dataclass
class Faults:
    drop_frames: list[int] = field(default_factory=list)          # frame indices (both cameras)
    drop_right_frames: list[int] = field(default_factory=list)    # right camera only
    stereo_offset_ns: dict[int, int] = field(default_factory=dict)  # frame index -> right offset
    imu_gaps: list[tuple[float, float]] = field(default_factory=list)  # (start_s, duration_s)
    late_frames: dict[int, int] = field(default_factory=dict)     # frame index -> extra latency ns
    loop_closure: tuple[float, float] | None = None   # (time_s, x shift m) applied to estimator
    estimator_reset_s: float | None = None            # estimator world restarts at origin


def _rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def motion(kind: str, t: np.ndarray):
    """Return (positions (N,3), yaw (N,)) in a z-up world for base_link (= IMU here)."""
    if kind == 'stationary':
        return np.zeros((t.size, 3)), np.zeros(t.size)
    if kind == 'loop':
        # Closed horizontal circle, radius 2 m, one revolution, nose tangent, with a gentle bob.
        T = t[-1] - t[0] if t.size > 1 else 1.0
        w = 2 * math.pi / T
        r = 2.0
        th = w * (t - t[0])
        p = np.stack([r * np.sin(th), r * (1 - np.cos(th)), 0.2 * np.sin(2 * th)], axis=1)
        return p, th
    raise ValueError(f'unknown motion {kind}')


def _stamp(msg_header, ns: int) -> None:
    msg_header.stamp.sec = int(ns // 1_000_000_000)
    msg_header.stamp.nanosec = int(ns % 1_000_000_000)


def _odom_msg(Odometry, ns, frame, child, T, v_body, w_body):
    m = Odometry()
    _stamp(m.header, ns)
    m.header.frame_id, m.child_frame_id = frame, child
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, T[:3, 3])
    q = matrix_to_quat_xyzw(T[:3, :3])
    o = m.pose.pose.orientation
    o.x, o.y, o.z, o.w = map(float, q)
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, v_body)
    m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = map(float, w_body)
    for i in range(3):
        m.pose.covariance[i * 7] = 1e-4
        m.pose.covariance[21 + i * 7] = 1e-5
        m.twist.covariance[i * 7] = 1e-3
    return m


def generate(out_dir, kind='loop', duration_s=20.0, cam_rate=30.0, imu_rate=200.0,
             faults: Faults | None = None, width=64, height=48, seed=0,
             latency_ns=8_000_000, estimator_noise_m=0.005) -> dict:
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import UInt64
    from x650_msgs.msg import FrameMetadata

    faults = faults or Faults()
    rng = np.random.default_rng(seed)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_path = out_dir / 'bag'
    w = BagWriter(bag_path)

    # Dense time grid for the analytic trajectory (IMU rate).
    t_imu = np.arange(0.0, duration_s, 1.0 / imu_rate)
    p, yaw = motion(kind, t_imu)
    dt = 1.0 / imu_rate
    vel_w = np.gradient(p, dt, axis=0)
    acc_w = np.gradient(vel_w, dt, axis=0)
    yaw_rate = np.gradient(np.unwrap(yaw), dt)

    def pose(i):
        T = np.eye(4)
        T[:3, :3] = _rot_z(yaw[i])
        T[:3, 3] = p[i]
        return T

    # IMU (FLU body == imu_link here): specific force and body rates.
    for i, ts in enumerate(t_imu):
        if any(a <= ts < a + d for a, d in faults.imu_gaps):
            continue
        R = _rot_z(yaw[i])
        f_body = R.T @ (acc_w[i] + np.array([0, 0, GRAVITY]))
        m = Imu()
        ns = T0_NS + int(round(ts * 1e9))
        _stamp(m.header, ns)
        m.header.frame_id = 'imu_link'
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(
            float, f_body + rng.normal(0, 0.02, 3))
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(
            float, np.array([0, 0, yaw_rate[i]]) + rng.normal(0, 0.002, 3))
        m.orientation_covariance[0] = -1.0  # orientation not provided (REP-145)
        w.write('/rig/imu', m, ns + 1_000_000)

    # Cameras, ground truth and estimator output at camera rate.
    n_frames = int(duration_s * cam_rate)
    gt_t, gt_T = [], []
    reset_count = 0
    reset_offset = None
    for k in range(n_frames):
        ts = k / cam_rate
        i = min(int(round(ts * imu_rate)), t_imu.size - 1)
        ns = T0_NS + int(round(ts * 1e9))
        T = pose(i)
        R = T[:3, :3]
        v_body = R.T @ vel_w[i]
        w_body = np.array([0, 0, yaw_rate[i]])
        gt_t.append(ns)
        gt_T.append(T)
        w.write('/groundtruth/odometry', _odom_msg(Odometry, ns, 'map', 'base_link', T, v_body, w_body), ns)

        # Estimator: ground truth + noise, with optional loop-closure shift / reset.
        Te = T.copy()
        Te[:3, 3] += rng.normal(0, estimator_noise_m, 3)
        if faults.loop_closure and ts >= faults.loop_closure[0]:
            Te[0, 3] += faults.loop_closure[1]
        if faults.estimator_reset_s is not None and ts >= faults.estimator_reset_s:
            if reset_offset is None:
                reset_offset = np.linalg.inv(T)
                reset_count += 1
                rc = UInt64()
                rc.data = reset_count
                w.write('/estimator/reset_count', rc, ns)
            Te = reset_offset @ Te
        w.write('/estimator/odometry',
                _odom_msg(Odometry, ns + 0, 'estimator_world', 'imu_link', Te, v_body, w_body),
                ns + latency_ns + 15_000_000)

        if k in faults.drop_frames:
            continue
        for cam in (0, 1):
            if cam == 1 and k in faults.drop_right_frames:
                continue
            cns = ns + (faults.stereo_offset_ns.get(k, 0) if cam == 1 else 0)
            img = Image()
            _stamp(img.header, cns)
            img.header.frame_id = f'cam{cam}_optical'
            img.height, img.width, img.encoding, img.step = height, width, 'mono8', width
            img.data = rng.integers(0, 255, width * height, dtype=np.uint8).tobytes()
            rec = cns + latency_ns + faults.late_frames.get(k, 0)
            w.write(f'/rig/cam{cam}/image_raw', img, rec)
            md = FrameMetadata()
            md.header = img.header
            md.camera_id = f'cam{cam}'
            md.sequence = k
            md.sensor_stamp_ns = cns - T0_NS
            md.clock_domain = 'synthetic'
            md.stamp_reference = 'exposure_start'
            md.exposure_us = 5000.0
            w.write(f'/rig/cam{cam}/frame_metadata', md, rec)
    w.close()

    gt = Trajectory(np.array(gt_t), np.array(gt_T))
    info = {
        'synthetic': True,
        'motion': kind,
        'duration_s': duration_s,
        'cam_rate_hz': cam_rate,
        'imu_rate_hz': imu_rate,
        'frames': n_frames,
        'seed': seed,
        'faults': {
            'drop_frames': faults.drop_frames,
            'drop_right_frames': faults.drop_right_frames,
            'stereo_offset_ns': {str(k): v for k, v in faults.stereo_offset_ns.items()},
            'imu_gaps': faults.imu_gaps,
            'late_frames': {str(k): v for k, v in faults.late_frames.items()},
            'loop_closure': faults.loop_closure,
            'estimator_reset_s': faults.estimator_reset_s,
        },
        'bag': str(bag_path),
        'groundtruth_samples': len(gt),
    }
    (out_dir / 'synthetic_info.json').write_text(json.dumps(info, indent=2))
    return info


def _ints(s):
    return [int(x) for x in s.split(',') if x] if s else []


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--motion', choices=['stationary', 'loop'], default='loop')
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--cam-rate', type=float, default=30.0)
    ap.add_argument('--imu-rate', type=float, default=200.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--drop-frames', default='')
    ap.add_argument('--drop-right-frames', default='')
    ap.add_argument('--imu-gap', action='append', default=[], help='start_s:duration_s')
    ap.add_argument('--loop-closure', default=None, help='time_s:x_shift_m')
    ap.add_argument('--estimator-reset', type=float, default=None, help='time_s')
    args = ap.parse_args(argv)
    f = Faults(
        drop_frames=_ints(args.drop_frames),
        drop_right_frames=_ints(args.drop_right_frames),
        imu_gaps=[tuple(map(float, g.split(':'))) for g in args.imu_gap],
        loop_closure=tuple(map(float, args.loop_closure.split(':'))) if args.loop_closure else None,
        estimator_reset_s=args.estimator_reset,
    )
    info = generate(args.out_dir, args.motion, args.duration, args.cam_rate, args.imu_rate, f, seed=args.seed)
    print(json.dumps(info, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
