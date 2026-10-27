import math

import numpy as np

from x650_sim.fake_vio import DelayLine, FakeVio, FakeVioConfig, inv_T, make_T, rot_z, yaw_of

MS = 1_000_000


def quiet(**kw):
    return FakeVio(FakeVioConfig(position_noise_m=0.0, drift_m_per_sqrt_s=0.0, **kw))


def pose(x, y, yaw):
    return make_T(rot_z(yaw), [x, y, 1.0])


def test_world_starts_at_first_pose_with_yaw_offset():
    v = quiet(yaw_offset_rad=0.6)
    s = v.process(0, pose(3.0, -2.0, 0.4), [0, 0, 0], [0, 0, 0])
    assert np.allclose(s.T_E_I[:3, 3], 0.0, atol=1e-12)
    assert math.isclose(yaw_of(s.T_E_I[:3, :3]), -0.6, abs_tol=1e-12)
    # Moving 1 m along the body x axis shows up rotated by the offset in E.
    s2 = v.process(40 * MS, pose(3.0 + math.cos(0.4), -2.0 + math.sin(0.4), 0.4), [0, 0, 0], [0, 0, 0])
    assert np.allclose(s2.T_E_I[:2, 3], [math.cos(-0.6), math.sin(-0.6)], atol=1e-12)


def test_rate_limit_and_dropout():
    v = quiet(rate_hz=30.0)
    assert v.process(0, pose(0, 0, 0), [0] * 3, [0] * 3) is not None
    assert v.process(10 * MS, pose(0, 0, 0), [0] * 3, [0] * 3) is None      # faster than 30 Hz
    v.command('dropout 0.5', 40 * MS)
    assert v.process(100 * MS, pose(0, 0, 0), [0] * 3, [0] * 3) is None
    assert v.process(600 * MS, pose(0, 0, 0), [0] * 3, [0] * 3) is not None


def test_jump_and_reset():
    v = quiet()
    v.process(0, pose(0, 0, 0), [0] * 3, [0] * 3)
    v.command('jump 0.5', 10 * MS)
    s = v.process(40 * MS, pose(0, 0, 0), [0] * 3, [0] * 3)
    assert math.isclose(s.T_E_I[0, 3], 0.5)
    v.command('reset', 50 * MS)
    s = v.process(80 * MS, pose(2.0, 1.0, 1.0), [0] * 3, [0] * 3)
    assert s.reset_count == 1
    assert np.allclose(s.T_E_I[:3, 3], 0.0, atol=1e-12)   # new world at the current pose


def test_latency_with_duration_reverts():
    v = quiet(latency_s=0.025)
    v.command('latency 0.15 1.0', 0)
    assert v.cfg.latency_s == 0.15
    v.process(500 * MS, pose(0, 0, 0), [0] * 3, [0] * 3)
    assert v.cfg.latency_s == 0.15
    v.process(1100 * MS, pose(0, 0, 0), [0] * 3, [0] * 3)
    assert v.cfg.latency_s == 0.025


def test_unknown_fault_reported():
    assert quiet().command('explode', 0).startswith('unknown')


def test_delay_line_fifo():
    d = DelayLine(maxlen=3)
    d.push(10, 'a')
    d.push(20, 'b')
    assert d.pop_ready(15) == ['a']
    assert d.pop_ready(25) == ['b']


def test_inv_T():
    T = pose(1.0, 2.0, 0.7)
    assert np.allclose(inv_T(T) @ T, np.eye(4))
