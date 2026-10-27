"""Integration: replay a synthetic session through the estimator adapter (requires a sourced
workspace with x650_estimator_adapter built). Checks global corrections and resets do not jump
local odometry, and that the report is produced."""
import shutil

import numpy as np
import pytest

pytest.importorskip('rosbag2_py')
pytest.importorskip('x650_msgs.msg')
if shutil.which('ros2') is None:
    pytest.skip('ros2 CLI not available', allow_module_level=True)
try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory('x650_estimator_adapter')
except Exception:
    pytest.skip('x650_estimator_adapter not installed', allow_module_level=True)

from x650_evaluation.evaluate import trajectory_from_bag  # noqa: E402
from x650_evaluation.replay import replay  # noqa: E402
from x650_evaluation.synthetic import Faults, generate  # noqa: E402


def _max_step(traj):
    return float(np.max(np.linalg.norm(np.diff(traj.positions, axis=0), axis=1)))


def test_replay_with_loop_closure_and_reset(tmp_path):
    info = generate(tmp_path / 's', duration_s=8.0,
                    faults=Faults(loop_closure=(3.0, 0.5), estimator_reset_s=6.0))
    rep = replay(info['bag'], tmp_path / 'out', reference_topic='/groundtruth/odometry',
                 sequence_type='handheld_closed_loop', rate=2.0)
    odom, _ = trajectory_from_bag(tmp_path / 'out' / 'replay_bag', '/vio/odometry')
    assert len(odom) >= info["frames"] - 2
    # 2 m radius loop over 8 s => ~5 cm per frame; a 50 cm jump would be obvious.
    assert _max_step(odom) < 0.1
    assert rep['health']['jumps_absorbed'] >= 2
    assert rep['health']['reset_count'] == 1
    assert rep['gates']['no_unhandled_resets'] is False  # resets are surfaced, not hidden
    assert rep['ate']['se3']['rmse_m'] < 0.2
