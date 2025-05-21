import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name='robot_core'
    
    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name),
            'launch',
            'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    #Launch Gazebo in ros_gz_sim package
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
            'launch',
            'ros_gz_sim.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
            'gz_args': ['-r -v 3 empty.sdf']
        }.items()
    )

    # Launch Gazebo bridge in ros_gz_bridge
    # Gazebo bridge config file
    gz_bridge_config = os.path.join(get_package_share_directory(package_name),
                                    'config', 'gz_bridge_params.yaml') 
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config    
        }],
        output='screen'
    )

    # Spawn model node in ros_gz_sim package
    gz_spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'robot_system_position',
            'true'
        ]
    )

    # Launch the file
    return LaunchDescription([
        rsp,
        gz_sim,
        gz_bridge,
        gz_spawn_model
    ])
