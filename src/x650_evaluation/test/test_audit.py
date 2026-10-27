"""Timestamp audit on synthetic sessions with injected faults (missing/late stereo, IMU gaps)."""
import pytest

pytest.importorskip('rosbag2_py')
x650_msgs = pytest.importorskip('x650_msgs.msg')

from x650_evaluation.synthetic import Faults, generate  # noqa: E402
from x650_evaluation.timestamp_audit import audit, check  # noqa: E402

PAIR = [('/rig/cam0/image_raw', '/rig/cam1/image_raw')]


def _audit(tmp_path, faults=None, **kw):
    info = generate(tmp_path / 's', duration_s=4.0, faults=faults, **kw)
    return audit(info['bag'], PAIR, '/rig/imu')


def test_clean_session_has_no_problems(tmp_path):
    rep = _audit(tmp_path)
    assert check(rep) == []
    cam = rep['topics']['/rig/cam0/image_raw']
    assert cam['count'] == 120 and abs(cam['rate_hz'] - 30.0) < 0.01
    assert rep['summary'] == {'frames_expected': 240, 'frames_recorded': 240,
                              'max_stereo_pair_offset_s': 0.0, 'imu_gaps': 0}
    lat = cam['record_minus_stamp_s']
    assert abs(lat['p50'] - 0.008) < 1e-6


def test_dropped_frames_detected_by_stamps_and_sequence(tmp_path):
    rep = _audit(tmp_path, Faults(drop_frames=[50, 51, 52]))
    cam = rep['topics']['/rig/cam0/image_raw']
    assert cam['gaps'] == 1 and cam['missing_estimate'] == 3
    assert rep['topics']['/rig/cam0/frame_metadata']['sequence']['dropped'] == 3
    assert rep['summary']['frames_expected'] - rep['summary']['frames_recorded'] == 6
    assert any('missing' in p for p in check(rep))


def test_missing_right_frame_and_stereo_offset(tmp_path):
    rep = _audit(tmp_path, Faults(drop_right_frames=[10], stereo_offset_ns={20: 2_000_000}))
    st = rep['stereo']['/rig/cam0/image_raw|/rig/cam1/image_raw']
    assert st['unpaired_left'] == 1 and st['unpaired_right'] == 0
    assert abs(st['max_abs_offset_s'] - 0.002) < 1e-9
    problems = check(rep)
    assert any('unpaired' in p for p in problems)
    assert any('pair offset' in p for p in problems)


def test_imu_gap_detected(tmp_path):
    rep = _audit(tmp_path, Faults(imu_gaps=[(1.0, 0.05)]))
    assert rep['imu']['gaps'] == 1
    assert abs(rep['imu']['max_gap_s'] - 0.055) < 0.006
    assert rep['summary']['imu_gaps'] == 1


def test_late_frame_shows_in_latency_not_in_stamps(tmp_path):
    rep = _audit(tmp_path, Faults(late_frames={30: 100_000_000}))
    cam = rep['topics']['/rig/cam0/image_raw']
    assert cam['gaps'] == 0  # acquisition stamps are unaffected by arrival time
    assert cam['record_minus_stamp_s']['max'] > 0.1
