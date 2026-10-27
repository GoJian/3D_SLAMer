"""Machine-readable evaluation report for one session (sprint items 4 and 6).

Reads an estimate (default /vio/odometry) and optionally a reference (bag topic or TUM file) and
writes JSON validated against config/schemas/evaluation_report.schema.json.

  x650_evaluate --bag out/replay_bag --reference-bag data/synthetic/loop/bag \
      --reference-topic /groundtruth/odometry --sequence-type handheld_closed_loop \
      --json reports/<session>_baseline.json

Latency: `record_minus_stamp_s` is (bag record time - header.stamp). For a live recording this is
capture-to-odometry age (plan gate: p95 < 50 ms). For a replay it also includes playback timing,
and the report says so via `mode`.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from x650_evaluation.bag import deserialize, header_stamp_ns, read_messages
from x650_evaluation.stats import percentiles
from x650_evaluation.trajectory import Trajectory, ate, closed_loop_drift, make_pose, read_tum, rpe

REPORT_VERSION = 1


def trajectory_from_bag(bag, topic) -> tuple[Trajectory, np.ndarray]:
    """Return (trajectory, record_minus_stamp seconds) from nav_msgs/Odometry messages."""
    t, T, lat = [], [], []
    last = None
    for m in read_messages(bag, [topic]):
        st = header_stamp_ns(m.data)
        if last is not None and st <= last:
            continue  # non-monotonic stamps are counted by the timestamp audit
        msg = deserialize(m)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t.append(st)
        T.append(make_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        lat.append((m.record_time_ns - st) * 1e-9)
        last = st
    if not t:
        raise ValueError(f'no messages on {topic} in {bag}')
    return Trajectory(np.array(t), np.array(T)), np.array(lat)


def health_from_bag(bag, topic='/vio/health') -> dict | None:
    last = None
    for m in read_messages(bag, [topic]):
        last = m
    if last is None:
        return None
    msg = deserialize(last)
    if not msg.status:
        return None
    s = msg.status[0]
    vals = {kv.key: kv.value for kv in s.values}
    out = {'level': int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level),
           'message': s.message}
    for k in ('reset_count', 'pose_revision', 'jumps_absorbed', 'accepted', 'rejected'):
        if k in vals:
            out[k] = int(vals[k])
    return out


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def evaluate(bag, topic='/vio/odometry', reference=None, reference_topic=None,
             sequence_type=None, mode='replay', session_id=None, rpe_delta_s=1.0,
             health_topic='/vio/health', synthetic=False, max_dt_s=0.01) -> dict:
    est, lat = trajectory_from_bag(bag, topic)
    duration = (est.t_ns[-1] - est.t_ns[0]) * 1e-9
    rate = (len(est) - 1) / duration if duration > 0 else None
    report = {
        'report_version': REPORT_VERSION,
        'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds'),
        'session_id': session_id,
        'synthetic': bool(synthetic),
        'mode': mode,
        'git_commit': _git_commit(),
        'sequence_type': sequence_type,
        'estimate': {'bag': str(bag), 'topic': topic, 'samples': len(est),
                     'duration_s': duration, 'rate_hz': rate},
        'latency': {'record_minus_stamp_s': percentiles(lat)},
        'health': health_from_bag(bag, health_topic),
    }
    if sequence_type in ('handheld_closed_loop', 'closed_loop', 'stationary'):
        report['closed_loop'] = closed_loop_drift(est)
    if reference is not None:
        if reference_topic:
            ref, _ = trajectory_from_bag(reference, reference_topic)
            ref_desc = {'bag': str(reference), 'topic': reference_topic}
        else:
            ref = read_tum(reference)
            ref_desc = {'tum': str(reference)}
        max_dt_ns = int(max_dt_s * 1e9)
        report['reference'] = ref_desc | {'samples': len(ref)}
        report['ate'] = {'se3': ate(est, ref, 'se3', max_dt_ns), 'sim3': ate(est, ref, 'sim3', max_dt_ns)}
        try:
            report['rpe'] = rpe(est, ref, rpe_delta_s, max_dt_ns)
        except ValueError as e:
            report['rpe'] = {'error': str(e)}
    report['gates'] = gates(report)
    return report


def gates(report: dict) -> dict:
    """Plan section 12 initial gates that can be judged from this report."""
    g = {}
    rate = report['estimate']['rate_hz']
    g['visual_rate_ge_30hz'] = None if rate is None else bool(rate >= 30.0 * 0.98)
    p95 = report['latency']['record_minus_stamp_s'].get('p95')
    g['p95_age_lt_50ms'] = None if p95 is None or report['mode'] != 'live' else bool(p95 < 0.050)
    h = report.get('health')
    g['no_unhandled_resets'] = None if h is None else bool(h.get('reset_count', 0) == 0)
    return g


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--topic', default='/vio/odometry')
    ap.add_argument('--reference-bag', default=None)
    ap.add_argument('--reference-topic', default=None)
    ap.add_argument('--reference-tum', default=None)
    ap.add_argument('--sequence-type', default=None)
    ap.add_argument('--mode', choices=['live', 'replay'], default='replay')
    ap.add_argument('--session-id', default=None)
    ap.add_argument('--synthetic', action='store_true')
    ap.add_argument('--rpe-delta-s', type=float, default=1.0)
    ap.add_argument('--json', default=None)
    args = ap.parse_args(argv)
    ref = args.reference_bag or args.reference_tum
    rep = evaluate(args.bag, args.topic, ref, args.reference_topic if args.reference_bag else None,
                   args.sequence_type, args.mode, args.session_id, args.rpe_delta_s,
                   synthetic=args.synthetic)
    text = json.dumps(rep, indent=2)
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
