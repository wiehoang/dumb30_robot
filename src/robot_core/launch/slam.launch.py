import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = 'robot_core'

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Config path
    slam_config = os.path.join(get_package_share_directory(pkg_name),
                               'config', 'online_async_slam.yaml')
    # Slam node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Sync and use simulator clock if true'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_config
        ),
        slam_node
    ])
