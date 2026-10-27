"""Automated PX4 SITL flight test with fault injection (M3).

  ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_faults
  ros2 run x650_sim x650_sim_flight_test --out data/sim/m3_nominal --faults ''

Starts sitl_vio.launch.py, records a bag, flies the offboard square mission (injecting faults at
waypoints), shuts everything down and writes report.json (x650_sim_evaluate). Exit code 0 only if
the mission completed and every applicable gate passed.

Default faults, by waypoint (0 = takeoff hover):
  1: jump 0.5         loop-closure-like 0.5 m jump in the estimator world
  2: dropout 1.0      estimator silent for 1 s
  3: reset            estimator restarts with a new world frame
  4: latency 0.15 1.0 estimator output 150 ms late for 1 s (bridge must reject it as stale)
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from x650_sim.sim_eval import evaluate

DEFAULT_FAULTS = '1:jump 0.5;2:dropout 1.0;3:reset;4:latency 0.15 1.0'
TOPICS = [
    '/groundtruth/odometry', '/sim/groundtruth_raw', '/estimator/odometry', '/estimator/reset_count', '/vio/odometry',
    '/vio/health', '/px4/odometry', '/px4_bridge/status', '/sim/fault', '/tf',
    '/fmu/in/vehicle_visual_odometry', '/fmu/out/vehicle_odometry',
    '/fmu/out/estimator_status_flags', '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_local_position_v1', '/fmu/out/timesync_status', '/fmu/out/sensor_combined',
]
SIM_PROCESSES = r'bin/px4 |gz sim|MicroXRCEAgent|parameter_bridge'


def _stop_group(p: subprocess.Popen, timeout=20.0) -> None:
    if p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()
    except ProcessLookupError:
        pass


def _leftovers() -> str:
    r = subprocess.run(['pgrep', '-af', SIM_PROCESSES], capture_output=True, text=True)
    return '\n'.join(l for l in r.stdout.splitlines() if 'pgrep' not in l)


def _wait_for(path: Path, text: str, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if path.exists() and text in path.read_text(errors='replace'):
            return True
        time.sleep(0.5)
    return False


def run(out, faults=DEFAULT_FAULTS, side_m=3.0, altitude_m=2.5, launch_args=(), time_source='sim') -> dict:
    out = Path(out).resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    if (left := _leftovers()):
        raise RuntimeError(f'simulation processes already running, stop them first:\n{left}')
    launch_log = out / 'launch.log'
    with open(launch_log, 'w') as ll, open(out / 'record.log', 'w') as rl, open(out / 'mission.log', 'w') as ml:
        launch = subprocess.Popen(['ros2', 'launch', 'x650_sim', 'sitl_vio.launch.py',
                                   f'px4_workdir:={out / "px4"}', f'time_source:={time_source}', *launch_args],
                                  stdout=ll, stderr=subprocess.STDOUT, start_new_session=True)
        rec = None
        mission_rc = None
        mission_ran = False
        try:
            if not _wait_for(launch_log, 'x650: PX4 parameters applied', 90):
                raise RuntimeError(f'PX4 did not come up, see {launch_log}')
            sim_flag = ['--use-sim-time'] if time_source == 'sim' else []
            rec = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', *sim_flag, '-o', str(out / 'bag'), *TOPICS],
                                   stdout=rl, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(3.0)
            cmd = ['ros2', 'run', 'x650_sim', 'x650_sim_mission', '--ros-args',
                   '-p', f'side_m:={side_m}', '-p', f'altitude_m:={altitude_m}',
                   '-p', f'use_sim_time:={"true" if time_source == "sim" else "false"}']
            if faults:
                cmd += ['-p', f'faults:={faults}']
            mission = subprocess.run(cmd, stdout=ml, stderr=subprocess.STDOUT, timeout=300)
            mission_rc = mission.returncode
            mission_ran = True
            time.sleep(2.0)
        except subprocess.TimeoutExpired:
            mission_rc = -1
        finally:
            if rec is not None:
                _stop_group(rec)
            _stop_group(launch)
    if not mission_ran and mission_rc is None:
        raise RuntimeError(f'run aborted before the mission, see {launch_log}')
    report = evaluate(out / 'bag')
    report['time_source'] = time_source
    report['mission'] = {'exit_code': mission_rc, 'faults': faults, 'side_m': side_m,
                         'altitude_m': altitude_m, 'log': str(out / 'mission.log')}
    report['gates']['mission_completed'] = mission_rc == 0
    report['passed'] = all(v is not False for v in report['gates'].values())
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--faults', default=DEFAULT_FAULTS)
    ap.add_argument('--side-m', type=float, default=3.0)
    ap.add_argument('--altitude-m', type=float, default=2.5)
    ap.add_argument('--time-source', choices=['sim', 'wall'], default='sim')
    ap.add_argument('launch_args', nargs='*', help='extra name:=value launch arguments')
    args = ap.parse_args(argv)
    rep = run(args.out, args.faults, args.side_m, args.altitude_m, args.launch_args, args.time_source)
    print(json.dumps({'gates': rep['gates'], 'passed': rep['passed'],
                      'report': str(Path(args.out) / 'report.json')}, indent=2))
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
