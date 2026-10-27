"""Trajectory alignment and error metrics (plan section 12).

Poses are (N, 4, 4) homogeneous matrices T_W_B; times are int64 nanoseconds.
SE(3) alignment is the default for metric systems. Sim(3) is available and must be reported
separately (it hides scale error).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    t_ns: np.ndarray        # (N,) int64
    T: np.ndarray           # (N, 4, 4) T_W_B

    def __post_init__(self):
        self.t_ns = np.asarray(self.t_ns, dtype=np.int64)
        self.T = np.asarray(self.T, dtype=float)
        if self.T.shape != (len(self.t_ns), 4, 4):
            raise ValueError('T must be (N,4,4) matching t_ns')
        if len(self.t_ns) > 1 and np.any(np.diff(self.t_ns) <= 0):
            raise ValueError('timestamps must be strictly increasing')

    @property
    def positions(self) -> np.ndarray:
        return self.T[:, :3, 3]

    def __len__(self) -> int:
        return len(self.t_ns)


def quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_pose(translation, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_matrix(quat_xyzw)
    T[:3, 3] = translation
    return T


def rotation_angle(R: np.ndarray) -> float:
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def associate(est: Trajectory, ref: Trajectory, max_dt_ns: int = 10_000_000):
    """Nearest-timestamp association. Returns index arrays (i_est, i_ref)."""
    idx = np.searchsorted(ref.t_ns, est.t_ns)
    i_est, i_ref = [], []
    for i, j in enumerate(idx):
        best, best_dt = None, None
        for k in (j - 1, j):
            if 0 <= k < len(ref):
                dt = abs(int(ref.t_ns[k]) - int(est.t_ns[i]))
                if best_dt is None or dt < best_dt:
                    best, best_dt = k, dt
        if best is not None and best_dt <= max_dt_ns:
            i_est.append(i)
            i_ref.append(best)
    return np.array(i_est, dtype=int), np.array(i_ref, dtype=int)


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False):
    """Least-squares (s, R, t) minimizing ||dst - (s R src + t)||. src, dst: (N, 3)."""
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError('need >= 3 matched points')
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = float(np.trace(np.diag(D) @ S) / xs.var(0).sum()) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def ate(est: Trajectory, ref: Trajectory, align: str = 'se3', max_dt_ns: int = 10_000_000) -> dict:
    """Absolute trajectory error after alignment. align: 'se3' | 'sim3' | 'none'."""
    i_e, i_r = associate(est, ref, max_dt_ns)
    if len(i_e) < 3:
        raise ValueError('fewer than 3 associated poses')
    pe, pr = est.positions[i_e], ref.positions[i_r]
    if align == 'none':
        s, R, t = 1.0, np.eye(3), np.zeros(3)
    else:
        s, R, t = umeyama(pe, pr, with_scale=(align == 'sim3'))
    err = np.linalg.norm(pr - (s * (R @ pe.T).T + t), axis=1)
    return {
        'alignment': align,
        'scale': s,
        'matched': int(len(i_e)),
        'rmse_m': float(np.sqrt(np.mean(err ** 2))),
        'mean_m': float(err.mean()),
        'median_m': float(np.median(err)),
        'max_m': float(err.max()),
    }


def rpe(est: Trajectory, ref: Trajectory, delta_s: float = 1.0, max_dt_ns: int = 10_000_000) -> dict:
    """Relative pose error over a fixed time delta (alignment-free)."""
    i_e, i_r = associate(est, ref, max_dt_ns)
    te = est.t_ns[i_e]
    delta_ns = int(delta_s * 1e9)
    trans, rot = [], []
    for a in range(len(i_e)):
        b = int(np.searchsorted(te, te[a] + delta_ns))
        if b >= len(i_e):
            break
        Ee = np.linalg.inv(est.T[i_e[a]]) @ est.T[i_e[b]]
        Er = np.linalg.inv(ref.T[i_r[a]]) @ ref.T[i_r[b]]
        E = np.linalg.inv(Er) @ Ee
        trans.append(np.linalg.norm(E[:3, 3]))
        rot.append(rotation_angle(E[:3, :3]))
    if not trans:
        raise ValueError('trajectory shorter than delta')
    trans, rot = np.array(trans), np.array(rot)
    return {
        'delta_s': delta_s,
        'pairs': int(len(trans)),
        'trans_rmse_m': float(np.sqrt(np.mean(trans ** 2))),
        'rot_rmse_deg': float(np.degrees(np.sqrt(np.mean(rot ** 2)))),
    }


def path_length(traj: Trajectory) -> float:
    return float(np.linalg.norm(np.diff(traj.positions, axis=0), axis=1).sum()) if len(traj) > 1 else 0.0


def closed_loop_drift(traj: Trajectory) -> dict:
    """For closed-loop sequences (start == end physically): end-to-start error."""
    d = traj.T[0, :3, 3] - traj.T[-1, :3, 3]
    L = path_length(traj)
    R = traj.T[0, :3, :3].T @ traj.T[-1, :3, :3]
    return {
        'end_position_error_m': float(np.linalg.norm(d)),
        'end_rotation_error_deg': float(np.degrees(rotation_angle(R))),
        'path_length_m': L,
        'drift_percent': float(100.0 * np.linalg.norm(d) / L) if L > 0 else None,
    }


def read_tum(path) -> Trajectory:
    """TUM format: t[s] x y z qx qy qz qw."""
    t, T = [], []
    for line in open(path):
        f = line.split()
        if not f or f[0].startswith('#'):
            continue
        t.append(_parse_seconds_ns(f[0]))  # exact: float64 epoch seconds lose ~100 ns
        v = [float(x) for x in f[1:8]]
        T.append(make_pose(v[0:3], v[3:7]))
    return Trajectory(np.array(t, dtype=np.int64), np.array(T))


def _parse_seconds_ns(s: str) -> int:
    if 'e' in s.lower():
        return int(round(float(s) * 1e9))
    whole, _, frac = s.partition('.')
    sign = -1 if whole.startswith('-') else 1
    return int(whole) * 1_000_000_000 + sign * int((frac + '000000000')[:9])


def matrix_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix to unit quaternion (x, y, z, w), w >= 0."""
    m = np.asarray(R, dtype=float)
    tr = np.trace(m)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def write_tum(traj: Trajectory, path) -> None:
    with open(path, 'w') as f:
        f.write('# t[s] x y z qx qy qz qw\n')
        for t, T in zip(traj.t_ns, traj.T):
            sec, nsec = divmod(int(t), 1_000_000_000)
            vals = ' '.join(f'{v:.9f}' for v in [*T[:3, 3], *matrix_to_quat_xyzw(T[:3, :3])])
            f.write(f'{sec}.{nsec:09d} {vals}\n')
