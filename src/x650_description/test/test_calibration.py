import shutil
from pathlib import Path

import numpy as np
import pytest

from x650_description.calibration import CalibrationError, compute_hash, load, static_transforms

HERE = Path(__file__).parent
FIXTURE = HERE / 'fixtures' / 'example_rig' / '20260101_synthetic'
SCHEMAS = HERE.parents[2] / 'config' / 'schemas'


def _R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_loads_and_verifies_hash():
    c = load(FIXTURE, SCHEMAS)
    assert c['calibration_hash'] == c['_computed_hash']


def test_finds_schemas_by_walking_up():
    assert load(FIXTURE)['rig_id'] == 'example_rig'


def test_static_transforms_chain_and_optical_axes():
    tfs = {(t.parent, t.child): t for t in static_transforms(load(FIXTURE, SCHEMAS))}
    assert set(tfs) == {('base_link', 'imu_link'), ('imu_link', 'cam0_optical'), ('imu_link', 'cam1_optical')}
    R = _R(tfs[('imu_link', 'cam0_optical')].rotation_xyzw)
    # Optical z (forward) is IMU +x; optical x (right) is IMU -y; optical y (down) is IMU -z.
    assert np.allclose(R @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0])
    assert np.allclose(R @ [0, 1, 0], [0, 0, -1])
    # Stereo baseline 12 cm, left camera on +y.
    t0 = np.array(tfs[('imu_link', 'cam0_optical')].translation)
    t1 = np.array(tfs[('imu_link', 'cam1_optical')].translation)
    assert np.isclose(np.linalg.norm(t0 - t1), 0.12) and t0[1] > t1[1]


def _copy(tmp_path):
    d = tmp_path / 'calib'
    shutil.copytree(FIXTURE, d)
    return d


def test_edit_in_place_is_rejected(tmp_path):
    d = _copy(tmp_path)
    idx = d / 'calibration.yaml'
    idx.write_text(idx.read_text().replace('rate_hz: 200.0', 'rate_hz: 400.0'))
    with pytest.raises(CalibrationError, match='never edited in place'):
        load(d, SCHEMAS)


def test_added_raw_file_changes_hash(tmp_path):
    d = _copy(tmp_path)
    before = compute_hash(d)
    (d / 'kalibr').mkdir()
    (d / 'kalibr' / 'camchain.yaml').write_text('cam0: {}\n')
    assert compute_hash(d) != before


def test_hash_ignores_its_own_field_and_comments(tmp_path):
    d = _copy(tmp_path)
    idx = d / 'calibration.yaml'
    idx.write_text('# extra comment\n' + idx.read_text())
    assert compute_hash(d) == compute_hash(FIXTURE)


def test_schema_and_quaternion_checks(tmp_path):
    d = _copy(tmp_path)
    idx = d / 'calibration.yaml'
    idx.write_text(idx.read_text().replace('rotation_xyzw: [0.0, 0.0, 0.0, 1.0]', 'rotation_xyzw: [0.0, 0.0, 0.0, 2.0]'))
    with pytest.raises(CalibrationError, match='unit norm'):
        load(d, SCHEMAS, verify_hash=False)
    idx.write_text(idx.read_text().replace('units: SI (m, s, rad)', 'units: mm'))
    with pytest.raises(CalibrationError, match='schema'):
        load(d, SCHEMAS, verify_hash=False)
