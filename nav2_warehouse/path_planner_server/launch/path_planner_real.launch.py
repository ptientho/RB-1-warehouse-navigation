from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    #get config file
    planner_config = os.path.join(get_package_share_directory('path_planner_server'),'config','real_robot','planner_config_real.yaml')
    controller_config = os.path.join(get_package_share_directory('path_planner_server'),'config','real_robot','controller_config_real.yaml')
    bt_config = os.path.join(get_package_share_directory('path_planner_server'),'config','real_robot','bt_navigator_config_real.yaml')
    behaviors_config = os.path.join(get_package_share_directory('path_planner_server'),'config','real_robot','behaviors_real.yaml')

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_config]

    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_config]
    
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behaviors_config]
    
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_config]
    
    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator']}]
    )

    return LaunchDescription([
        controller_node,
        behavior_node,
        bt_navigator_node,
        planner_node,
        lifecycle_node
    
    ])