import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_name = 'robot_core'
    
    # Config args
    autostart = LaunchConfiguration('autostart') 
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Config path
    slam_config = os.path.join(get_package_share_directory(pkg_name),
                               'config', 'online_async_slam.yaml')
    # Slam node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        namespace=''
    )
    
    """
    # If use slam_toolbox with nav2, the code below will be ignored
    because nav2 has its own lifecycle manager to bring up all components
    (slam_toolbox included also)
    
    # If use slam_toolbox alone, it need to setup its own lifecycle manager
    by changing state from unconfigure -> configure -> activate -> inactive
    """

    configure_event_handler = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))) # true + not(false) = true
    )

    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=(
                LogInfo(msg='[LifecycleManager] slam node is activating'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            )
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))) 
    ) 

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Start slam_toolbox alone'
        ),
        DeclareLaunchArgument(
            'use_lifecycle_manager', default_value='false',
            description='Set to true if start slam_toolbox with nav2 package'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Sync and use simulator clock if true'
        ),
        DeclareLaunchArgument(
            'slam_params_file', default_value=slam_config
        ),
        slam_node,
        configure_event_handler,
        activate_event_handler
    ])
