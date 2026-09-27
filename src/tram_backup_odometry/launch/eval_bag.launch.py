"""Launch the estimator together with a rosbag2 replay (demo / jury check).

    ros2 launch tram_backup_odometry eval_bag.launch.py bag:=/path/to/bag
    ros2 launch tram_backup_odometry eval_bag.launch.py bag:=<bag> rate:=2.0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tram_backup_odometry')
    default_params = os.path.join(share, 'config', 'tram_params.yaml')

    args = [
        DeclareLaunchArgument('bag', description='path to the rosbag2 directory'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('gnss_init_window_s', default_value='60.0'),
        DeclareLaunchArgument('init_mode', default_value='auto'),
        DeclareLaunchArgument('start_delay', default_value='2.0'),
        DeclareLaunchArgument('score', default_value='true',
                              description='run the offline scorer after the replay'),
    ]

    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
             '--rate', LaunchConfiguration('rate'),
             '--loop', LaunchConfiguration('loop')],
        output='screen',
    )

    node = Node(
        package='tram_backup_odometry',
        executable='tram_backup_odometry_node',
        name='tram_backup_odometry',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'route.gnss_init_window_s': LaunchConfiguration('gnss_init_window_s'),
            'route.init_mode': LaunchConfiguration('init_mode'),
        }],
        emulate_tty=True,
    )

    score = ExecuteProcess(
        cmd=['ros2', 'run', 'tram_backup_odometry', 'evaluate_bag',
             LaunchConfiguration('bag')],
        output='screen',
    )

    return LaunchDescription(args + [
        node,
        TimerAction(period=float(LaunchConfiguration('start_delay').default[0]
                                 if hasattr(LaunchConfiguration('start_delay'), 'default')
                                 else 2.0),
                    actions=[play]),
        TimerAction(period=600.0, actions=[score],
                    condition=IfCondition(LaunchConfiguration('score'))),
    ])
