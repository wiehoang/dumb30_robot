import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package name
    pkg_name='robot_core'
   
    # Config paths
    gz_bridge_config = os.path.join(get_package_share_directory(pkg_name),
                                    'config', 'gz_bridge_params.yaml') # Gazebo bridge config
    rviz_config = os.path.join(get_package_share_directory(pkg_name),
                               'config', 'robot_view.rviz') # Rviz config
    ros2_control_config = os.path.join(get_package_share_directory(pkg_name),
                                       'config', 'robot_controller.yaml') # Ros2_control config
 
    # Launch robot_state_publisher
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name),
            'launch',
            'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'gz_args': [' -r -v 3 empty.sdf']
        }.items()
    )

    # Gazebo bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config    
        }],
        output='screen'
    )

    # Spawn model node
    gz_spawn_model_node = Node(
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

    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_control_config],
        output='both'
    )

    # joint_state_broadcaster node
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    # diff_drive_controller node
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--param-file', ros2_control_config]
    )

    # Launch the file
    return LaunchDescription([
        robot_state_pub,
        gz_sim,
        gz_bridge_node,
        gz_spawn_model_node,
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster,
        diff_drive_controller
    ])
