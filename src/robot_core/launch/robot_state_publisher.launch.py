import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_core'))
    xacro_file = os.path.join(pkg_path,'model','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    
    # Launch the file
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description="Set to true to sync time with Gazebo"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_config,
                "use_sim_time": use_sim_time
            }]
        )
    ])
