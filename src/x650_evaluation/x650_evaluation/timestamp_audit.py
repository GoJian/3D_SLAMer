"""Timestamp audit for a recorded session (M1 exit criterion).

Checks, per topic: message count, rate, stamp gaps (estimated dropped frames), duplicate stamps,
messages arriving out of acquisition order, and record-minus-stamp latency. Across topics: stereo pair offsets and unpaired
frames, IMU gaps, and sequence-number gaps from x650_msgs/FrameMetadata.

Acquisition time is header.stamp. Bag record time is used only for latency statistics.

  x650_timestamp_audit data/sessions/<id>/bag --stereo /rig/cam0/image_raw,/rig/cam1/image_raw \
      --imu /rig/imu --json audit.json
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict

import numpy as np

from x650_evaluation.bag import HEADER_FIRST_TYPES, deserialize, header_stamp_ns, read_messages
from x650_evaluation.stats import percentiles

DEFAULT_GAP_FACTOR = 1.5


def _interval_stats(stamps: np.ndarray, gap_factor: float) -> dict:
    """stamps in arrival (record) order. Gaps are computed in acquisition order; arrival-order
    inversions are reported separately as `reordered`."""
    out = {'count': int(stamps.size)}
    if stamps.size < 2:
        return out | {'rate_hz': None, 'gaps': 0, 'missing_estimate': 0,
                      'reordered': 0, 'duplicates': 0}
    reordered = int(np.sum(np.diff(stamps) < 0))
    d = np.diff(np.sort(stamps))
    pos = d[d > 0]
    nominal = float(np.median(pos)) if pos.size else 0.0
    gaps = pos[pos > gap_factor * nominal] if nominal > 0 else np.array([])
    missing = int(np.sum(np.round(gaps / nominal) - 1)) if gaps.size else 0
    duration = (stamps.max() - stamps.min()) * 1e-9
    return out | {
        'duration_s': float(duration),
        'rate_hz': float(1e9 / nominal) if nominal > 0 else None,
        'mean_rate_hz': float((stamps.size - 1) / duration) if duration > 0 else None,
        'nominal_interval_s': nominal * 1e-9,
        'gaps': int(gaps.size),
        'max_gap_s': float(gaps.max() * 1e-9) if gaps.size else 0.0,
        'missing_estimate': missing,
        'reordered': reordered,
        'duplicates': int(np.sum(d == 0)),
    }


def pair_stereo(left: np.ndarray, right: np.ndarray, tolerance_ns: int) -> dict:
    """Pair each left stamp with the nearest right stamp (one-to-one, within tolerance)."""
    left, right = np.sort(left), np.sort(right)
    offsets, used = [], set()
    unpaired_left = 0
    for t in left:
        j = int(np.searchsorted(right, t))
        best = None
        for k in (j - 1, j):
            if 0 <= k < right.size and k not in used:
                if best is None or abs(right[k] - t) < abs(right[best] - t):
                    best = k
        if best is not None and abs(int(right[best]) - int(t)) <= tolerance_ns:
            used.add(best)
            offsets.append(int(right[best]) - int(t))
        else:
            unpaired_left += 1
    off = np.abs(np.array(offsets, dtype=float)) * 1e-9
    return {
        'pairs': len(offsets),
        'unpaired_left': unpaired_left,
        'unpaired_right': int(right.size - len(used)),
        'abs_offset_s': percentiles(off),
        'max_abs_offset_s': float(off.max()) if off.size else None,
    }


def sequence_stats(seqs: list[int]) -> dict:
    s = np.array(seqs, dtype=np.int64)
    if s.size < 2:
        return {'count': int(s.size), 'dropped': 0, 'duplicates': 0, 'out_of_order': 0}
    d = np.diff(s)
    return {
        'count': int(s.size),
        'dropped': int(np.sum(d[d > 1] - 1)),
        'duplicates': int(np.sum(d == 0)),
        'out_of_order': int(np.sum(d < 0)),
    }


def audit(bag_path, stereo_pairs=(), imu_topic=None, gap_factor=DEFAULT_GAP_FACTOR,
          stereo_tolerance_s=0.005, imu_gap_factor=None) -> dict:
    stamps: dict[str, list[int]] = defaultdict(list)
    latency: dict[str, list[float]] = defaultdict(list)
    seqs: dict[str, list[int]] = defaultdict(list)
    types: dict[str, str] = {}
    counts: dict[str, int] = defaultdict(int)
    for m in read_messages(bag_path):
        types[m.topic] = m.type
        counts[m.topic] += 1
        if m.type in HEADER_FIRST_TYPES:
            st = header_stamp_ns(m.data)
            stamps[m.topic].append(st)
            latency[m.topic].append((m.record_time_ns - st) * 1e-9)
        if m.type == 'x650_msgs/msg/FrameMetadata':
            seqs[m.topic].append(int(deserialize(m).sequence))

    topics = {}
    for topic, typ in sorted(types.items()):
        entry = {'type': typ, 'messages': counts[topic]}
        if topic in stamps:
            arr = np.array(stamps[topic], dtype=np.int64)
            entry |= _interval_stats(arr, gap_factor)
            entry['record_minus_stamp_s'] = percentiles(latency[topic])
        if topic in seqs:
            entry['sequence'] = sequence_stats(seqs[topic])
        topics[topic] = entry

    report = {'bag': str(bag_path), 'topics': topics, 'stereo': {}, 'imu': None}
    for left, right in stereo_pairs:
        if left not in stamps or right not in stamps:
            report['stereo'][f'{left}|{right}'] = {'error': 'topic missing'}
            continue
        report['stereo'][f'{left}|{right}'] = pair_stereo(
            np.array(stamps[left]), np.array(stamps[right]), int(stereo_tolerance_s * 1e9))
    if imu_topic:
        if imu_topic in stamps:
            report['imu'] = _interval_stats(np.array(stamps[imu_topic], dtype=np.int64),
                                            imu_gap_factor or gap_factor)
        else:
            report['imu'] = {'error': 'topic missing'}
    report['summary'] = summarize(report)
    return report


def summarize(report: dict) -> dict:
    """Fields matching session_manifest.schema.json `audit`."""
    cams = [t for t, e in report['topics'].items() if e['type'] == 'sensor_msgs/msg/Image']
    recorded = sum(report['topics'][t].get('count', 0) for t in cams)
    missing = sum(report['topics'][t].get('missing_estimate', 0) for t in cams)
    offsets = [v.get('max_abs_offset_s') for v in report['stereo'].values() if v.get('max_abs_offset_s') is not None]
    return {
        'frames_expected': int(recorded + missing),
        'frames_recorded': int(recorded),
        'max_stereo_pair_offset_s': float(max(offsets)) if offsets else 0.0,
        'imu_gaps': int((report['imu'] or {}).get('gaps', 0)),
    }


def check(report: dict, max_pair_offset_s: float = 0.0005) -> list[str]:
    """Return a list of human-readable failures against M1 expectations."""
    problems = []
    for t, e in report['topics'].items():
        if e.get('reordered'):
            problems.append(f'{t}: {e["reordered"]} messages arrived out of acquisition order')
        if e.get('duplicates'):
            problems.append(f'{t}: {e["duplicates"]} duplicate stamps')
        if e.get('missing_estimate'):
            problems.append(f'{t}: ~{e["missing_estimate"]} missing messages in {e["gaps"]} gaps')
        if e.get('sequence', {}).get('dropped'):
            problems.append(f'{t}: {e["sequence"]["dropped"]} sequence numbers dropped')
    for k, v in report['stereo'].items():
        if 'error' in v:
            problems.append(f'stereo {k}: {v["error"]}')
            continue
        if v['unpaired_left'] or v['unpaired_right']:
            problems.append(f'stereo {k}: {v["unpaired_left"]} left / {v["unpaired_right"]} right unpaired')
        if v['max_abs_offset_s'] is not None and v['max_abs_offset_s'] > max_pair_offset_s:
            problems.append(f'stereo {k}: max pair offset {v["max_abs_offset_s"]*1e3:.3f} ms '
                            f'> {max_pair_offset_s*1e3:.3f} ms')
    imu = report.get('imu')
    if imu and imu.get('gaps'):
        problems.append(f'imu: {imu["gaps"]} gaps (max {imu["max_gap_s"]*1e3:.1f} ms)')
    if imu and 'error' in imu:
        problems.append(f'imu: {imu["error"]}')
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--stereo', action='append', default=[],
                    help='left,right image topics (repeatable)')
    ap.add_argument('--imu', default=None)
    ap.add_argument('--gap-factor', type=float, default=DEFAULT_GAP_FACTOR)
    ap.add_argument('--stereo-tolerance-s', type=float, default=0.005,
                    help='max |offset| to count as a pair')
    ap.add_argument('--max-pair-offset-s', type=float, default=0.0005,
                    help='sync target (config/clock_policy.yaml sync_target_s)')
    ap.add_argument('--json', default=None, help='write full report here')
    args = ap.parse_args(argv)
    pairs = [tuple(s.split(',')) for s in args.stereo]
    report = audit(args.bag, pairs, args.imu, args.gap_factor, args.stereo_tolerance_s)
    problems = check(report, args.max_pair_offset_s)
    report['problems'] = problems
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(report, f, indent=2)
    print(json.dumps(report['summary'], indent=2))
    for p in problems:
        print('PROBLEM', p)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
