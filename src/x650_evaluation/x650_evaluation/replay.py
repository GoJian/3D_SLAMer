"""Replay a session through the estimator adapter and evaluate (sprint item 4).

  x650_replay data/synthetic/loop/bag --out data/replays/loop \
      --reference-topic /groundtruth/odometry --sequence-type handheld_closed_loop

Steps: start `ros2 bag record` for the adapter outputs, start the adapter with use_sim_time,
`ros2 bag play --clock`, stop, then write report.json with x650_evaluate.

The estimator itself (OKVIS2-X) is not wired yet; until it is, the input is whatever publishes
nav_msgs/Odometry on --estimator-topic in the bag (synthetic sessions provide /estimator/odometry).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from x650_evaluation.evaluate import evaluate

OUTPUT_TOPICS = ['/vio/odometry', '/vio/health', '/tf']


def _stop(p: subprocess.Popen, timeout=10.0):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


def replay(bag, out, estimator_topic='/estimator/odometry', reset_topic='/estimator/reset_count',
           calibration=None, rate=1.0, reference_topic=None, sequence_type=None, session_id=None,
           synthetic=None) -> dict:
    out = Path(out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    rec_bag = out / 'replay_bag'
    env = dict(os.environ)
    # Isolate replays from anything else on the network.
    env.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    logs = open(out / 'replay.log', 'w')
    adapter_cmd = ['ros2', 'launch', 'x650_estimator_adapter', 'estimator_adapter.launch.py',
                   f'estimator_odometry:={estimator_topic}', f'estimator_reset_count:={reset_topic}',
                   'use_sim_time:=true']
    if calibration:
        adapter_cmd.append(f'calibration:={calibration}')
    rec = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '--use-sim-time', '-o', str(rec_bag),
                            *OUTPUT_TOPICS], env=env, stdout=logs, stderr=subprocess.STDOUT)
    adapter = subprocess.Popen(adapter_cmd, env=env, stdout=logs, stderr=subprocess.STDOUT)
    try:
        time.sleep(3.0)  # discovery; topics are subscribed before playback starts
        play = subprocess.run(['ros2', 'bag', 'play', str(bag), '--clock', '100', '-r', str(rate),
                               '--disable-keyboard-controls', '--delay', '2.0'],
                              env=env, stdout=logs, stderr=subprocess.STDOUT)
        time.sleep(1.0)
    finally:
        _stop(adapter)
        _stop(rec)
        logs.close()
    if play.returncode != 0:
        raise RuntimeError(f'bag play failed, see {out / "replay.log"}')
    if synthetic is None:
        synthetic = (Path(bag).parent / 'synthetic_info.json').exists()
    report = evaluate(rec_bag, reference=bag if reference_topic else None,
                      reference_topic=reference_topic, sequence_type=sequence_type, mode='replay',
                      session_id=session_id, synthetic=synthetic)
    report['replay'] = {'input_bag': str(bag), 'estimator_topic': estimator_topic, 'rate': rate,
                        'calibration': calibration}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', required=True)
    ap.add_argument('--estimator-topic', default='/estimator/odometry')
    ap.add_argument('--reset-topic', default='/estimator/reset_count')
    ap.add_argument('--calibration', default=None)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--reference-topic', default=None)
    ap.add_argument('--sequence-type', default=None)
    ap.add_argument('--session-id', default=None)
    args = ap.parse_args(argv)
    rep = replay(args.bag, args.out, args.estimator_topic, args.reset_topic, args.calibration,
                 args.rate, args.reference_topic, args.sequence_type, args.session_id)
    print(json.dumps(rep, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
