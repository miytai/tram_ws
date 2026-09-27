"""Launch the backup-odometry node with the calibrated configuration."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tram_backup_odometry')
    default_params = os.path.join(share, 'config', 'tram_params.yaml')
    default_map = os.path.join(share, 'config', 'route_map.json')
    default_model = os.path.join(share, 'config', 'model.json')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('model_file', default_value=default_model),
        DeclareLaunchArgument('gnss_init_window_s', default_value='60.0'),
        DeclareLaunchArgument('init_mode', default_value='auto',
                              description='auto | absolute | relative'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='tram_backup_odometry',
            executable='tram_backup_odometry_node',
            name='tram_backup_odometry',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'model.model_file': LaunchConfiguration('model_file'),
                'route.map_file': LaunchConfiguration('map_file'),
                'route.gnss_init_window_s': LaunchConfiguration('gnss_init_window_s'),
                'route.init_mode': LaunchConfiguration('init_mode'),
                'route.use_map': LaunchConfiguration('use_map'),
                'output.publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
        ),
    ])
