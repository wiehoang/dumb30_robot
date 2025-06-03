import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # Package name
    pkg_name='robot_core'

    # Config paths
    # config robot_description
    xacro_file = os.path.join(get_package_share_directory(pkg_name),
                              'model','robot.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file]) # Process URDF (xacro) to get robot_description string

    gz_bridge_config = os.path.join(get_package_share_directory(pkg_name),
                                    'config', 'gz_bridge_params.yaml') # Gazebo bridge config
    rviz_config = os.path.join(get_package_share_directory(pkg_name),
                               'rviz', 'robot_view.rviz') # Rviz config
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'my_bedroom.sdf') 
    
    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # robot_state_publisher node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo environment
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gz_args': [' -r -v3 ', world],
            'on_exit_shutdown': 'true'
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
            '-topic', 'robot_description',
            '-entity_name', 'dumb30_robot',
            '-allow_renaming', 'true',
            '-z', '0.1',
        ]
    )

    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # joint_state_broadcaster node
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    # diff_drive_controller node
    diff_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--controller-ros-args', '-r diff_drive_base_controller/cmd_vel:=/cmd_vel',
        ],
    )

    # Launch the file
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'world', default_value=world_path,
            description='SDF world file'
        ),
        gz_sim,
        robot_state_pub,
        gz_spawn_model_node,
        gz_bridge_node,
        joint_state_broadcaster,
        diff_drive_base_controller,
        rviz_node,
    ])
