"""Manifest built from a synthetic session validates and snapshots config + calibration."""
import json
import shutil
from pathlib import Path

import pytest

pytest.importorskip('rosbag2_py')
pytest.importorskip('x650_msgs.msg')
pytest.importorskip('x650_evaluation')
if shutil.which('ros2') is None:
    pytest.skip('ros2 CLI not available', allow_module_level=True)

from x650_evaluation.synthetic import Faults, generate  # noqa: E402
from x650_recording.record import main  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
CALIB = REPO / 'src' / 'x650_description' / 'test' / 'fixtures' / 'example_rig' / '20260101_synthetic'


def test_finalize_synthetic_session(tmp_path, monkeypatch):
    session = tmp_path / 'sessions' / 'synthetic_stationary'
    generate(session, kind='stationary', duration_s=3.0, faults=Faults(drop_frames=[10]))
    # rosbag2 SequentialWriter writes metadata.yaml on close.
    assert (session / 'bag' / 'metadata.yaml').exists()
    monkeypatch.chdir(REPO)
    assert main(['--finalize-only', '--sequence-type', 'stationary', '--session-id',
                 'synthetic_stationary', '--out-root', str(tmp_path / 'sessions'),
                 '--calibration', str(CALIB), '--notes', 'synthetic']) == 0
    m = json.loads((session / 'session_manifest.json').read_text())
    assert m['calibration_id'] == '20260101_synthetic'
    assert m['calibration_hash'].startswith('sha256:')
    assert m['bag']['storage'] == 'mcap'
    assert '/rig/imu' in m['bag']['topics']
    assert m['audit']['frames_expected'] - m['audit']['frames_recorded'] == 2
    assert (session / 'calibration' / 'calibration.yaml').exists()
    assert (session / 'config' / 'clock_policy.yaml').exists()
    assert (session / 'timestamp_audit.json').exists()
