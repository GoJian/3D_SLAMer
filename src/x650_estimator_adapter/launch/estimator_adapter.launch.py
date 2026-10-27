"""Launch the estimator adapter.

  ros2 launch x650_estimator_adapter estimator_adapter.launch.py \
      estimator_odometry:=/okvis/odometry calibration:=calibration/rigs/<rig>/<id>/calibration.yaml

If `calibration` is given, T_base_imu is read from it (overriding the YAML placeholder).
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = [str(Path(get_package_share_directory('x650_estimator_adapter')) / 'config' / 'estimator_adapter.yaml')]
    calib = LaunchConfiguration('calibration').perform(context)
    if calib:
        T = yaml.safe_load(Path(calib).read_text())['T_base_imu']
        params.append({
            'T_base_imu.translation': [float(v) for v in T['translation']],
            'T_base_imu.rotation_xyzw': [float(v) for v in T['rotation_xyzw']],
        })
    params.append({'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'})
    return [Node(
        package='x650_estimator_adapter',
        executable='estimator_adapter_node',
        name='estimator_adapter',
        parameters=params,
        remappings=[('~/input/odometry', LaunchConfiguration('estimator_odometry')),
                    ('~/input/reset_count', LaunchConfiguration('estimator_reset_count'))],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('estimator_odometry', default_value='/estimator/odometry'),
        DeclareLaunchArgument('estimator_reset_count', default_value='/estimator/reset_count'),
        DeclareLaunchArgument('calibration', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
