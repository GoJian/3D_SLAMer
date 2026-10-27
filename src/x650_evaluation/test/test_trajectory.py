import numpy as np
import pytest

from x650_evaluation.trajectory import (Trajectory, ate, closed_loop_drift, make_pose,
                                        matrix_to_quat_xyzw, quat_xyzw_to_matrix, read_tum, rpe,
                                        umeyama, write_tum)


def _random_rot(rng):
    q = rng.normal(size=4)
    return quat_xyzw_to_matrix(q / np.linalg.norm(q))


def _traj(n=200, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(n, dtype=np.int64) * 33_000_000 + 10**18
    T = np.stack([make_pose(rng.normal(size=3) * 3, rng.normal(size=4)) for _ in range(n)])
    return Trajectory(t, T)


def test_quaternion_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        R = _random_rot(rng)
        assert np.allclose(quat_xyzw_to_matrix(matrix_to_quat_xyzw(R)), R, atol=1e-12)


@pytest.mark.parametrize('with_scale', [False, True])
def test_umeyama_recovers_transform(with_scale):
    rng = np.random.default_rng(2)
    src = rng.normal(size=(100, 3))
    R, t, s = _random_rot(rng), rng.normal(size=3), 1.7 if with_scale else 1.0
    dst = s * src @ R.T + t
    s2, R2, t2 = umeyama(src, dst, with_scale)
    assert np.allclose(R2, R) and np.allclose(t2, t) and np.isclose(s2, s)


def test_ate_zero_after_rigid_misalignment_and_sim3_reports_scale():
    ref = _traj()
    G = make_pose([1, 2, 3], [0.1, 0.2, 0.3, 0.9])
    est = Trajectory(ref.t_ns, np.einsum('ij,njk->nik', G, ref.T))
    assert ate(est, ref, 'se3')['rmse_m'] < 1e-9
    assert ate(est, ref, 'none')['rmse_m'] > 0.1
    scaled = Trajectory(ref.t_ns, ref.T.copy())
    scaled.T[:, :3, 3] *= 1.1
    se3, sim3 = ate(scaled, ref, 'se3'), ate(scaled, ref, 'sim3')
    assert sim3['rmse_m'] < 1e-9 and np.isclose(sim3['scale'], 1 / 1.1)
    assert se3['rmse_m'] > 0.05  # SE(3) exposes the scale error


def test_rpe_zero_for_identical_and_nonzero_for_drift():
    ref = _traj()
    assert rpe(ref, ref, 1.0)['trans_rmse_m'] < 1e-12
    drift = Trajectory(ref.t_ns, ref.T.copy())
    drift.T[:, 0, 3] += np.linspace(0, 1, len(ref))
    assert rpe(drift, ref, 1.0)['trans_rmse_m'] > 0.01


def test_association_tolerance():
    ref = _traj()
    est = Trajectory(ref.t_ns + 20_000_000, ref.T)  # 20 ms off
    with pytest.raises(ValueError):
        ate(est, ref, 'se3', max_dt_ns=10_000_000)


def test_closed_loop_drift():
    n = 100
    th = np.linspace(0, 2 * np.pi, n)
    T = np.stack([make_pose([np.cos(a), np.sin(a), 0], [0, 0, 0, 1]) for a in th])
    T[-1, 0, 3] += 0.1
    r = closed_loop_drift(Trajectory(np.arange(n) * 10**7, T))
    assert np.isclose(r['end_position_error_m'], 0.1, atol=1e-9)
    assert 6.0 < r['path_length_m'] < 6.5


def test_tum_round_trip(tmp_path):
    ref = _traj(20)
    write_tum(ref, tmp_path / 't.txt')
    back = read_tum(tmp_path / 't.txt')
    assert np.allclose(back.T, ref.T, atol=1e-8)
    assert np.array_equal(back.t_ns, ref.t_ns)


def test_rejects_non_increasing_time():
    with pytest.raises(ValueError):
        Trajectory(np.array([1, 1]), np.stack([np.eye(4)] * 2))
