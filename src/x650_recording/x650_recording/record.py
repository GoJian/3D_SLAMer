"""Record a session (M1): rosbag2 MCAP + snapshot + timestamp audit + manifest.

  x650_record --sequence-type stationary --calibration calibration/rigs/<rig>/<id>
  # Ctrl-C to stop. Output: data/sessions/<UTC>_<sequence_type>/

Recording runs `ros2 bag record` with MCAP storage and a bounded writer cache. Acquisition time is
header.stamp from the camera wrapper; bag receive time is kept only for latency analysis.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import signal
import subprocess
import sys
from pathlib import Path

import yaml

from x650_recording.manifest import build, find_repo_root


def default_topics_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('x650_recording')) / 'config' / 'record_topics.yaml'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'config' / 'record_topics.yaml'


def record_command(bag_dir: Path, topics_cfg: dict) -> list[str]:
    return ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
            '--max-cache-size', str(int(topics_cfg.get('max_cache_size_bytes', 1 << 30))),
            *topics_cfg['topics']]


def finalize(session_dir: Path, session_id: str, topics_cfg: dict, args, repo: Path) -> dict:
    audit_summary = None
    problems = []
    try:
        from x650_evaluation.timestamp_audit import audit, check
        rep = audit(session_dir / 'bag', [tuple(p) for p in topics_cfg.get('stereo_pairs', [])],
                    topics_cfg.get('imu_topic'))
        problems = check(rep)
        rep['problems'] = problems
        (session_dir / 'timestamp_audit.json').write_text(json.dumps(rep, indent=2) + '\n')
        audit_summary = rep['summary']
    except ImportError:
        print('x650_evaluation not available; skipping audit', file=sys.stderr)
    manifest = build(
        session_dir, session_id,
        hardware_revision_file=args.hardware or repo / 'config' / 'hardware' / 'x650_rev0.yaml',
        calibration_dir=args.calibration,
        clock_policy_file=args.clock_policy or repo / 'config' / 'clock_policy.yaml',
        dependency_lock_file=args.dependency_lock or repo / 'config' / 'dependencies.jetson.lock.yaml',
        sequence_type=args.sequence_type, audit_summary=audit_summary, notes=args.notes, repo=repo)
    for p in problems:
        print('AUDIT PROBLEM', p, file=sys.stderr)
    return manifest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sequence-type', required=True,
                    help='stationary | handheld_closed_loop | flight | ...')
    ap.add_argument('--calibration', default=None, help='calibration directory (required for real sessions)')
    ap.add_argument('--topics', default=None, help='topic config YAML')
    ap.add_argument('--out-root', default=None, help='default: <repo>/data/sessions')
    ap.add_argument('--session-id', default=None)
    ap.add_argument('--hardware', default=None)
    ap.add_argument('--clock-policy', default=None)
    ap.add_argument('--dependency-lock', default=None)
    ap.add_argument('--notes', default='')
    ap.add_argument('--finalize-only', action='store_true',
                    help='bag already recorded in <session>/bag; only audit + manifest')
    args = ap.parse_args(argv)

    repo = find_repo_root(Path.cwd()) or find_repo_root(Path(__file__).resolve())
    if repo is None:
        ap.error('run from inside the repository (config/schemas not found)')
    topics_cfg = yaml.safe_load(Path(args.topics or default_topics_file()).read_text())
    session_id = args.session_id or (
        dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + f'_{args.sequence_type}')
    session_dir = Path(args.out_root or repo / 'data' / 'sessions') / session_id

    if not args.finalize_only:
        if session_dir.exists():
            ap.error(f'{session_dir} exists')
        session_dir.mkdir(parents=True)
        cmd = record_command(session_dir / 'bag', topics_cfg)
        print('recording:', ' '.join(cmd), '\nCtrl-C to stop', file=sys.stderr)
        proc = subprocess.Popen(cmd)
        try:
            proc.wait()
        except KeyboardInterrupt:
            proc.send_signal(signal.SIGINT)
            proc.wait()
    manifest = finalize(session_dir, session_id, topics_cfg, args, repo)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
