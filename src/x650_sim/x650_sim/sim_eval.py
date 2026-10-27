"""Evaluate a PX4 SITL flight-test recording against the M3 exit criteria.

  x650_sim_evaluate data/sim/<run>/bag --json data/sim/<run>/report.json

M3 exit (docs/milestones.md): correct axes and timing; stale-data behavior works; a global
correction does not jump local odometry. Checks, all on the host clock:

  axes        PX4 EKF output (/px4/odometry, converted back to REP-103) vs the odometry we sent
              (/vio/odometry), with NO alignment: same frame by construction if the axes are right.
  timing      time lag that best aligns PX4's output with Gazebo ground truth (should be ~0).
  accuracy    SE(3)-aligned ATE of /px4/odometry and /vio/odometry vs ground truth.
  continuity  largest position step not explained by the reported velocity, near injected
              jumps/resets, in /vio/odometry and /px4/odometry.
  stale       EV samples older than the bridge's max age are never sent to PX4.
  fusion      fraction of in-air EKF samples with EV position fusion active.

Simulation with a fake VIO: this measures the integration, not estimator performance.
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np

from x650_evaluation.bag import deserialize, read_messages
from x650_evaluation.evaluate import trajectory_from_bag
from x650_evaluation.stats import percentiles
from x650_evaluation.trajectory import Trajectory, associate, ate, rotation_angle

GT, VIO, PX4 = '/groundtruth/odometry', '/vio/odometry', '/px4/odometry'
EV_IN = '/fmu/in/vehicle_visual_odometry'
FLAGS = '/fmu/out/estimator_status_flags'
FAULT = '/sim/fault'

GATES = {
    'axes_max_rms_m': 0.15,          # unaligned PX4 vs sent odometry
    'axes_max_yaw_p95_deg': 5.0,
    'timing_max_abs_lag_s': 0.020,
    'ate_max_rmse_m': 0.10,          # sim with ~cm-level fake VIO noise
    'continuity_max_step_m': 0.05,   # 10% of the default 0.5 m injected jump
    'fusion_min_fraction': 0.90,
}


def fault_events(bag) -> list[dict]:
    return [{'t_ns': m.record_time_ns, 'command': deserialize(m).data}
            for m in read_messages(bag, [FAULT])]


def best_lag(est: Trajectory, ref: Trajectory, max_lag_s=0.15, step_s=0.005) -> dict:
    """Lag (s) added to est stamps that minimizes SE(3)-aligned ATE; positive = est stamped early."""
    best = None
    for lag in np.arange(-max_lag_s, max_lag_s + 1e-9, step_s):
        shifted = Trajectory(est.t_ns + int(round(lag * 1e9)), est.T)
        try:
            r = ate(shifted, ref, 'se3', 10_000_000)['rmse_m']
        except ValueError:
            continue
        if best is None or r < best[1]:
            best = (float(lag), r)
    if best is None:
        return {'lag_s': None}
    edge = abs(abs(best[0]) - max_lag_s) < step_s / 2
    return {'lag_s': best[0], 'rmse_at_lag_m': best[1], 'at_search_edge': edge}


def _xcorr_lag(t_a, a, t_b, b, max_lag_s=0.06, step_s=0.001):
    """Lag (s) to add to series a's stamps so it best matches series b (least squares)."""
    lo, hi = max(t_a[0], t_b[0]) + max_lag_s, min(t_a[-1], t_b[-1]) - max_lag_s
    if hi - lo < 1.0:
        return None
    grid = np.arange(lo, hi, step_s)
    B = np.interp(grid, t_b, b)
    errs = [(float(np.mean((np.interp(grid - lag, t_a, a) - B) ** 2)), float(lag))
            for lag in np.arange(-max_lag_s, max_lag_s + 1e-9, step_s)]
    return min(errs)[1]


def imu_lag(bag) -> dict:
    """Lag of PX4's IMU (sensor_combined, host-converted stamps) vs ground-truth yaw rate.

    Isolates PX4-internal timing (e.g. SITL stamping IMU on arrival) from our EV chain."""
    tg, wg, ti, wi = [], [], [], []
    for m in read_messages(bag, [GT]):
        o = deserialize(m)
        tg.append(o.header.stamp.sec + o.header.stamp.nanosec * 1e-9)
        wg.append(o.twist.twist.angular.z)
    for m in read_messages(bag, ['/fmu/out/sensor_combined']):
        o = deserialize(m)
        ti.append(o.timestamp * 1e-6)
        wi.append(-o.gyro_rad[2])          # FRD z -> FLU z
    if len(tg) < 100 or len(ti) < 100:
        return {'lag_s': None}
    oi, og = np.argsort(ti), np.argsort(tg)
    lag = _xcorr_lag(np.array(ti)[oi], np.array(wi)[oi], np.array(tg)[og], np.array(wg)[og])
    return {'lag_s': lag, 'samples': len(ti)}


def unaligned_diff(a: Trajectory, b: Trajectory, max_dt_ns=20_000_000) -> dict:
    ia, ib = associate(a, b, max_dt_ns)
    if len(ia) < 3:
        return {'matched': int(len(ia))}
    d = np.linalg.norm(a.positions[ia] - b.positions[ib], axis=1)
    yaw = np.degrees([rotation_angle(a.T[i, :3, :3].T @ b.T[j, :3, :3]) for i, j in zip(ia, ib)])
    return {'matched': int(len(ia)), 'position_rms_m': float(np.sqrt(np.mean(d ** 2))),
            'position_max_m': float(d.max()), 'rotation_deg': percentiles(yaw)}


def step_residuals(bag, topic) -> tuple[np.ndarray, np.ndarray]:
    """(t_ns[1:], |dp - R v dt|) using the message's own body-frame velocity."""
    t, p, v = [], [], []
    from x650_evaluation.trajectory import quat_xyzw_to_matrix
    last = None
    for m in read_messages(bag, [topic]):
        o = deserialize(m)
        st = o.header.stamp.sec * 1_000_000_000 + o.header.stamp.nanosec
        if last is not None and st <= last:
            continue
        last = st
        q = o.pose.pose.orientation
        R = quat_xyzw_to_matrix([q.x, q.y, q.z, q.w])
        lv = o.twist.twist.linear
        t.append(st)
        p.append([o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z])
        v.append(R @ np.array([lv.x, lv.y, lv.z]))
    t, p, v = np.array(t), np.array(p), np.array(v)
    if len(t) < 2:
        return np.array([]), np.array([])
    dt = np.diff(t) * 1e-9
    pred = 0.5 * (v[1:] + v[:-1]) * dt[:, None]
    return t[1:], np.linalg.norm(np.diff(p, axis=0) - pred, axis=1)


def continuity(bag, topic, events, window_s=1.5) -> dict:
    t, r = step_residuals(bag, topic)
    if r.size == 0:
        return {'samples': 0}
    out = {'samples': int(r.size), 'max_step_residual_m': float(r.max()),
           'p99_step_residual_m': float(np.percentile(r, 99)), 'near_events': []}
    for e in events:
        if e['command'].split()[0] not in ('jump', 'reset', 'dropout', 'latency'):
            continue
        sel = (t >= e['t_ns'] - 0.2e9) & (t <= e['t_ns'] + window_s * 1e9)
        out['near_events'].append({'command': e['command'],
                                   'max_step_residual_m': float(r[sel].max()) if sel.any() else None})
    return out


def ev_stats(bag, max_age_s) -> dict:
    ages, stamps = [], []
    for m in read_messages(bag, [EV_IN]):
        o = deserialize(m)
        ages.append((int(o.timestamp) - int(o.timestamp_sample)) * 1e-6)
        stamps.append(int(o.timestamp_sample))
    if not ages:
        return {'messages': 0}
    a = np.array(ages)
    s = np.sort(np.array(stamps)) * 1e-6
    gaps = np.diff(s) if s.size > 1 else np.array([0.0])
    return {'messages': len(ages), 'send_age_s': percentiles(a),
            'older_than_max_age': int(np.sum(a > max_age_s)), 'max_gap_s': float(gaps.max()),
            'rate_hz': float((s.size - 1) / (s[-1] - s[0])) if s.size > 1 and s[-1] > s[0] else None}


def outage_windows(events, max_age_s, recovery_s=1.0) -> list[tuple[int, int]]:
    """Intended vision outages (dropout, latency above the bridge max age) plus recovery time."""
    out = []
    for e in events:
        p = e['command'].split()
        if p[0] == 'dropout':
            d = float(p[1]) if len(p) > 1 else 1.0
        elif p[0] == 'latency' and float(p[1]) > max_age_s and len(p) > 2:
            d = float(p[2])
        else:
            continue
        out.append((e['t_ns'], e['t_ns'] + int((d + recovery_s) * 1e9)))
    return out


def fusion_stats(bag, windows=()) -> dict:
    n_air = n_air_ev = n_avail = n_avail_ev = 0
    transitions, prev = [], None
    for m in read_messages(bag, [FLAGS]):
        o = deserialize(m)
        if o.cs_in_air:
            n_air += 1
            n_air_ev += bool(o.cs_ev_pos)
            if not any(a <= m.record_time_ns <= b for a, b in windows):
                n_avail += 1
                n_avail_ev += bool(o.cs_ev_pos)
        state = (bool(o.cs_ev_pos), bool(o.cs_ev_yaw), bool(o.cs_ev_hgt), bool(o.cs_ev_vel))
        if state != prev:
            transitions.append({'t_ns': m.record_time_ns, 'ev_pos': state[0], 'ev_yaw': state[1],
                                'ev_hgt': state[2], 'ev_vel': state[3]})
            prev = state
    return {'in_air_samples': n_air,
            'in_air_ev_pos_fraction': (n_air_ev / n_air) if n_air else None,
            'in_air_outside_outages_samples': n_avail,
            'in_air_outside_outages_ev_pos_fraction': (n_avail_ev / n_avail) if n_avail else None,
            'transitions': transitions}


def last_diag(bag, topic) -> dict:
    last = None
    for m in read_messages(bag, [topic]):
        last = m
    if last is None:
        return {}
    msg = deserialize(last)
    return {kv.key: kv.value for s in msg.status for kv in s.values} | \
        {'message': msg.status[0].message if msg.status else None}


def evaluate(bag, max_age_s=0.1) -> dict:
    gt, _ = trajectory_from_bag(bag, GT)
    vio, vio_lat = trajectory_from_bag(bag, VIO)
    px4, _ = trajectory_from_bag(bag, PX4)
    events = fault_events(bag)
    t0 = int(gt.t_ns[0])
    rep = {
        'simulation': True,
        'note': 'PX4 SITL + fake VIO from Gazebo ground truth; integration test, not estimator performance',
        'samples': {'groundtruth': len(gt), 'vio': len(vio), 'px4': len(px4)},
        'faults': [{'t_s': (e['t_ns'] - t0) * 1e-9, 'command': e['command']} for e in events],
        'axes': unaligned_diff(px4, vio),
        'timing': {'px4_vs_groundtruth': best_lag(px4, gt), 'vio_vs_groundtruth': best_lag(vio, gt),
                   'px4_imu_vs_groundtruth': imu_lag(bag),
                   'vio_record_minus_stamp_s': percentiles(vio_lat)},
        'ate': {'px4': ate(px4, gt, 'se3', 20_000_000), 'vio': ate(vio, gt, 'se3', 20_000_000)},
        'continuity': {'vio': continuity(bag, VIO, events), 'px4': continuity(bag, PX4, events)},
        'ev_input': ev_stats(bag, max_age_s),
        'fusion': fusion_stats(bag, outage_windows(events, max_age_s)),
        'adapter_health': last_diag(bag, '/vio/health'),
        'bridge_status': last_diag(bag, '/px4_bridge/status'),
    }
    for tr in rep['fusion']['transitions']:
        tr['t_s'] = (tr.pop('t_ns') - t0) * 1e-9
    rep['gates'] = gates(rep, events)
    rep['passed'] = all(v is not False for v in rep['gates'].values())
    return rep


def gates(rep, events) -> dict:
    g = {}
    ax = rep['axes']
    g['axes'] = ('position_rms_m' in ax and ax['position_rms_m'] < GATES['axes_max_rms_m']
                 and ax['rotation_deg']['p95'] < GATES['axes_max_yaw_p95_deg'])
    lag = rep['timing']['px4_vs_groundtruth']
    g['timing'] = lag.get('lag_s') is not None and not lag['at_search_edge'] and \
        abs(lag['lag_s']) <= GATES['timing_max_abs_lag_s']
    g['accuracy'] = rep['ate']['px4']['rmse_m'] < GATES['ate_max_rmse_m']
    # Gate only on global corrections (jump/reset). After a vision outage PX4 legitimately
    # corrects its IMU-only drift; that step is reported but not gated.
    near = [e['max_step_residual_m'] for k in ('vio', 'px4')
            for e in rep['continuity'][k].get('near_events', [])
            if e['command'].split()[0] in ('jump', 'reset')]
    g['local_continuity'] = None if not near else all(
        x is not None and x < GATES['continuity_max_step_m'] for x in near)
    ev = rep['ev_input']
    stale_fault = any(e['command'].startswith('latency') for e in events)
    g['no_stale_sent'] = ev.get('messages', 0) > 0 and ev['older_than_max_age'] == 0
    g['stale_rejected'] = None if not stale_fault else \
        int(rep['bridge_status'].get('count_stale', 0)) > 0
    frac = rep['fusion']['in_air_outside_outages_ev_pos_fraction']
    g['ev_fused_in_flight'] = frac is not None and frac >= GATES['fusion_min_fraction']
    resets = sum(e['command'].startswith('reset') for e in events)
    g['resets_absorbed'] = None if not resets else \
        int(rep['adapter_health'].get('reset_count', 0)) == resets
    return g


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--max-age-s', type=float, default=0.1)
    ap.add_argument('--json', default=None)
    args = ap.parse_args(argv)
    rep = evaluate(args.bag, args.max_age_s)
    text = json.dumps(rep, indent=2)
    if args.json:
        with open(args.json, 'w') as f:
            f.write(text + '\n')
    print(text)
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
