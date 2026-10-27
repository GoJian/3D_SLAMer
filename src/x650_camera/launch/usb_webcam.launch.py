"""One or two USB webcams on the /rig/cam_i topics.

  ros2 launch x650_camera usb_webcam.launch.py                          # /dev/video0 -> /rig/cam0
  ros2 launch x650_camera usb_webcam.launch.py cam1_device:=/dev/video2 # second webcam -> /rig/cam1

Two webcams are free-running (no hardware sync); the timestamp audit reports their offset.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    cfg = str(Path(get_package_share_directory('x650_camera')) / 'config' / 'usb_webcam.yaml')
    nodes = []
    for i in (0, 1):
        dev = LaunchConfiguration(f'cam{i}_device').perform(context)
        if not dev:
            continue
        overrides = {'video_device': dev, 'camera_id': f'cam{i}', 'frame_id': f'cam{i}_optical'}
        exp = float(LaunchConfiguration('exposure_us').perform(context))
        if exp > 0:
            overrides['exposure_us'] = exp
        nodes.append(Node(package='x650_camera', executable='usb_camera_node', name=f'cam{i}',
                          namespace='rig', parameters=[cfg, overrides], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam0_device', default_value='/dev/video0'),
        DeclareLaunchArgument('cam1_device', default_value='', description='empty = mono'),
        DeclareLaunchArgument('exposure_us', default_value='-1.0'),
        OpaqueFunction(function=_nodes),
    ])
