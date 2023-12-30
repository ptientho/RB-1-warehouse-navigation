from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    #get config file
    planner_config = os.path.join(get_package_share_directory('path_planner_server'),'config','planner_config.yaml')
    controller_config = os.path.join(get_package_share_directory('path_planner_server'),'config','controller_config.yaml')
    bt_config = os.path.join(get_package_share_directory('path_planner_server'),'config','bt_navigator_config.yaml')
    behaviors_config = os.path.join(get_package_share_directory('path_planner_server'),'config','behaviors.yaml')
    rviz_config = os.path.join(get_package_share_directory('path_planner_server'),'rviz','path_planner.rviz')

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
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator']}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    
    )


    return LaunchDescription([
        
        rviz_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_node
    
    ])