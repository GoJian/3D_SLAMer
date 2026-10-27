"""PX4 SITL (Gazebo Harmonic, headless) + fake VIO + estimator adapter + PX4 bridge.

  ros2 launch x650_sim sitl_vio.launch.py                 # PX4 at $PX4_DIR or ~/px4/PX4-Autopilot

time_source:=sim (default): ROS nodes run on Gazebo's /clock and PX4 timestamp sync is off
(UXRCE_DDS_SYNCT=0), so PX4 time == ROS time. time_source:=wall: ROS on the host clock with uXRCE
timesync, as on hardware; in lockstep SITL this showed a ~18 ms bias (decision 0004).

Data path:
  gz OdometryPublisher -> ros_gz_bridge -> fake_vio -> /estimator/odometry
  -> estimator_adapter -> /vio/odometry -> px4_bridge -> /fmu/in/vehicle_visual_odometry
  -> MicroXRCEAgent (UDP 8888) -> PX4 EKF2.  PX4's EKF output returns as /px4/odometry.
"""
import os
import shutil
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = Path(get_package_share_directory('x650_sim'))
    px4_dir = Path(os.path.expanduser(LaunchConfiguration('px4_dir').perform(context)))
    px4_bin = px4_dir / 'build' / 'px4_sitl_default' / 'bin' / 'px4'
    if not px4_bin.exists():
        raise RuntimeError(f'{px4_bin} not found; run scripts/setup_px4_sitl.sh')
    model = LaunchConfiguration('model').perform(context)
    workdir = LaunchConfiguration('px4_workdir').perform(context) or tempfile.mkdtemp(prefix='px4_sitl_')
    workdir = str(Path(os.path.expanduser(workdir)).resolve())
    Path(workdir).mkdir(parents=True, exist_ok=True)

    params = yaml.safe_load((share / 'config' / 'px4_sitl_ev_params.yaml').read_text())['px4_params']
    sim_time = LaunchConfiguration('time_source').perform(context) == 'sim'
    params['UXRCE_DDS_SYNCT'] = 0 if sim_time else 1
    env = {f'PX4_PARAM_{k}': str(v) for k, v in params.items()}
    env.update({
        'PX4_SYS_AUTOSTART': LaunchConfiguration('autostart').perform(context),
        'PX4_SIM_MODEL': f'gz_{model}',
        'PX4_GZ_WORLD': LaunchConfiguration('world').perform(context),
        'HEADLESS': '1',
    })

    # Gazebo is started here (not by PX4) so that shutting down the launch also stops it.
    world = LaunchConfiguration('world').perform(context)
    gz_dir = px4_dir / 'Tools' / 'simulation' / 'gz'
    gz_env = {
        'GZ_IP': '127.0.0.1',
        # ROS sets GZ_CONFIG_PATH to its vendor packages, which lack `gz sim`; use gz-harmonic's.
        'GZ_CONFIG_PATH': '/usr/share/gz',
        'GZ_SIM_RESOURCE_PATH': f"{gz_dir / 'models'}:{gz_dir / 'worlds'}",
        'GZ_SIM_SYSTEM_PLUGIN_PATH': str(px4_dir / 'build' / 'px4_sitl_default' / 'src' / 'modules' / 'simulation' / 'gz_plugins'),
        'GZ_SIM_SERVER_CONFIG_PATH': str(px4_dir / 'src' / 'modules' / 'simulation' / 'gz_bridge' / 'server.config'),
    }
    # /usr/bin/gz (gz-harmonic); the ROS gz_tools_vendor 'gz' on PATH has no 'sim' command.
    gz_cmd = [shutil.which('gz', path='/usr/bin') or 'gz', 'sim', '--verbose=1', '-r']
    if LaunchConfiguration('headless').perform(context) == 'true':
        gz_cmd.append('-s')
    gazebo = ExecuteProcess(cmd=gz_cmd + [str(gz_dir / 'worlds' / f'{world}.sdf')],
                            additional_env=gz_env, output='log', name='gazebo', sigterm_timeout='5')
    env.update(gz_env)
    env['PX4_GZ_STANDALONE'] = '1'
    env['PX4_GZ_MODELS'] = str(gz_dir / 'models')
    env['PX4_GZ_WORLDS'] = str(gz_dir / 'worlds')

    px4 = ExecuteProcess(
        cmd=[str(px4_bin), '-d', '-w', workdir, str(px4_dir / 'build' / 'px4_sitl_default' / 'etc')],
        cwd=workdir, additional_env=env, output='screen', name='px4', sigterm_timeout='5')
    # PX4_PARAM_* env vars are applied before the airframe script, whose `param set-default` can
    # still change a value we set equal to the old default (e.g. NAV_DLL_ACT 0 -> 2 for x500).
    # Re-apply every parameter through the px4-param client once the PX4 server is up.
    bin_dir = px4_bin.parent
    sets = ' && '.join(f'{bin_dir}/px4-param set {k} {v}' for k, v in params.items())
    px4_params = ExecuteProcess(
        cmd=['bash', '-c',
             f'for i in $(seq 120); do {bin_dir}/px4-param show SYS_AUTOSTART >/dev/null 2>&1 && break; '
             f'sleep 0.5; done; sleep 2; {sets} && echo "x650: PX4 parameters" "applied"'],
        cwd=workdir, output='screen', name='px4_params')
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log', name='xrce_agent')
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_odometry_bridge',
        arguments=[f'/model/{model}_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']
        + (['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'] if sim_time else []),
        remappings=[(f'/model/{model}_0/odometry', '/sim/groundtruth_raw')],
        additional_env={'GZ_IP': '127.0.0.1'}, output='log')
    fake_vio = Node(
        package='x650_sim', executable='fake_vio', name='fake_vio',
        parameters=[{'yaw_offset_rad': float(LaunchConfiguration('vio_yaw_offset_rad').perform(context)),
                     'latency_s': float(LaunchConfiguration('vio_latency_s').perform(context)),
                     'stamp_source': 'message' if sim_time else 'arrival',
                     'use_sim_time': sim_time}],
        remappings=[('~/input/groundtruth', '/sim/groundtruth_raw')], output='screen')
    adapter = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('x650_estimator_adapter')) / 'launch' / 'estimator_adapter.launch.py')),
        launch_arguments={'use_sim_time': 'true' if sim_time else 'false'}.items())
    bridge = Node(
        package='x650_px4_bridge', executable='px4_bridge_node', name='px4_bridge',
        parameters=[str(Path(get_package_share_directory('x650_px4_bridge')) / 'config' / 'px4_bridge.yaml'),
                    {'use_sim_time': sim_time, 'require_timesync': not sim_time}],
        output='screen')
    return [gazebo, agent, px4, px4_params, gz_bridge, fake_vio, adapter, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', '~/px4/PX4-Autopilot')),
        DeclareLaunchArgument('px4_workdir', default_value='', description='empty = fresh temp dir'),
        DeclareLaunchArgument('model', default_value='x500_vision'),
        DeclareLaunchArgument('autostart', default_value='4005'),
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('vio_yaw_offset_rad', default_value='0.6'),
        DeclareLaunchArgument('vio_latency_s', default_value='0.025'),
        DeclareLaunchArgument('time_source', default_value='sim', choices=['sim', 'wall']),
        OpaqueFunction(function=_setup),
    ])
