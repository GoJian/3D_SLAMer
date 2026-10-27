import numpy as np

from x650_sim.sim_eval import _xcorr_lag, outage_windows


def test_xcorr_lag_sign():
    t = np.arange(0, 20, 0.005)
    f = np.sin(1.3 * t) + 0.5 * np.sin(3.1 * t)
    # a stamped 18 ms late -> shift its stamps by -18 ms to match b
    assert abs(_xcorr_lag(t + 0.018, f, t, f) + 0.018) < 1.5e-3
    assert abs(_xcorr_lag(t - 0.010, f, t, f) - 0.010) < 1.5e-3


def test_outage_windows():
    s = 1_000_000_000
    ev = [{'t_ns': 10 * s, 'command': 'dropout 1.0'},
          {'t_ns': 20 * s, 'command': 'latency 0.15 1.0'},
          {'t_ns': 30 * s, 'command': 'latency 0.05 1.0'},   # below max age: not an outage
          {'t_ns': 40 * s, 'command': 'jump 0.5'}]
    w = outage_windows(ev, max_age_s=0.1, recovery_s=1.0)
    assert w == [(10 * s, 12 * s), (20 * s, 22 * s)]
