from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    map_name = 'rb1_warehouse.yaml'
    map_file = os.path.join(get_package_share_directory('map_server'),'config',map_name)

    amcl_config = 'amcl_config.yaml'
    amcl_yaml = os.path.join(get_package_share_directory('localization_server'),'config',amcl_config)
    
    rviz_config_file = 'localization_rb1.rviz'

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename':map_file}]
    
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]

    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server','amcl']}]
    )
    
    rviz_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', os.path.join(get_package_share_directory('localization_server'),'rviz',rviz_config_file)]
    
    )

    return LaunchDescription([
        
        map_server_node,
        amcl_node,
        lifecycle_node,
        
    ])