"""Static TF from a versioned calibration.

  ros2 launch x650_description description.launch.py calibration:=calibration/rigs/<rig>/<id>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration'),
        DeclareLaunchArgument('schema_dir', default_value=''),
        Node(package='x650_description', executable='calibration_tf_publisher',
             parameters=[{'calibration': LaunchConfiguration('calibration'),
                          'schema_dir': LaunchConfiguration('schema_dir')}],
             output='screen'),
    ])
