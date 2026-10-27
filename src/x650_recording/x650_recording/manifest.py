"""Session manifest (config/schemas/session_manifest.schema.json).

A session directory is self-contained for replay:

  <session_dir>/
    bag/                       rosbag2 MCAP
    session_manifest.json
    config/                    snapshot: hardware revision, clock policy, dependency lock
    calibration/               snapshot of the calibration directory used
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import yaml


def sha256_file(path) -> str:
    return 'sha256:' + hashlib.sha256(Path(path).read_bytes()).hexdigest()


def find_repo_root(start: Path) -> Path | None:
    for d in [start, *start.parents]:
        if (d / 'config' / 'schemas' / 'session_manifest.schema.json').exists():
            return d
    return None


def git_state(repo: Path) -> tuple[str | None, bool | None]:
    try:
        commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True,
                                         stderr=subprocess.DEVNULL).strip()
        dirty = bool(subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain'],
                                             text=True, stderr=subprocess.DEVNULL).strip())
        return commit, dirty
    except Exception:
        return None, None


def bag_info(bag_dir: Path) -> dict:
    """Read rosbag2 metadata.yaml (written when recording stops)."""
    meta = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns = int(meta['starting_time']['nanoseconds_since_epoch'])
    return {
        'storage': meta['storage_identifier'],
        'start_ns': start_ns,
        'duration_s': int(meta['duration']['nanoseconds']) * 1e-9,
        'topics': sorted(t['topic_metadata']['name'] for t in meta['topics_with_message_count']),
        'message_count': int(meta['message_count']),
    }


def validate(manifest: dict, repo: Path) -> None:
    from jsonschema import Draft202012Validator
    schema = json.loads((repo / 'config' / 'schemas' / 'session_manifest.schema.json').read_text())
    errors = list(Draft202012Validator(schema).iter_errors(manifest))
    if errors:
        raise ValueError('; '.join(f"{'/'.join(map(str, e.path))}: {e.message}" for e in errors))


def build(session_dir, session_id, hardware_revision_file, calibration_dir, clock_policy_file,
          dependency_lock_file, sequence_type=None, audit_summary=None, notes='',
          repo=None, snapshot=True) -> dict:
    session_dir = Path(session_dir)
    repo = Path(repo) if repo else find_repo_root(Path(__file__).resolve()) or find_repo_root(Path.cwd())
    if repo is None:
        raise RuntimeError('repository root (config/schemas) not found; pass repo=')
    info = bag_info(session_dir / 'bag')
    hw = yaml.safe_load(Path(hardware_revision_file).read_text())

    calib_id = calib_hash = None
    if calibration_dir:
        from x650_description.calibration import load
        calib = load(calibration_dir, repo / 'config' / 'schemas')
        calib_id, calib_hash = calib['calibration_id'], calib['calibration_hash']
    commit, dirty = git_state(repo)

    if snapshot:
        cfg = session_dir / 'config'
        cfg.mkdir(exist_ok=True)
        for f in (hardware_revision_file, clock_policy_file, dependency_lock_file):
            shutil.copy2(f, cfg / Path(f).name)
        if calibration_dir:
            shutil.copytree(calibration_dir, session_dir / 'calibration', dirs_exist_ok=True)

    start = dt.datetime.fromtimestamp(info['start_ns'] * 1e-9, dt.timezone.utc)
    manifest = {
        'session_id': session_id,
        'start_time_utc': start.isoformat().replace('+00:00', 'Z'),
        'duration_s': info['duration_s'],
        'hardware_revision': hw['hardware_revision'],
        'calibration_id': calib_id or 'none',
        'calibration_hash': calib_hash or 'none',
        'clock_policy': sha256_file(clock_policy_file),
        'dependency_lock_hash': sha256_file(dependency_lock_file),
        'git_commit': (commit or 'unknown') + ('-dirty' if dirty else ''),
        'bag': {'path': 'bag', 'storage': info['storage'], 'topics': info['topics']},
        'notes': notes,
    }
    if sequence_type:
        manifest['sequence_type'] = sequence_type
    if audit_summary:
        manifest['audit'] = audit_summary
    validate(manifest, repo)
    (session_dir / 'session_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest
