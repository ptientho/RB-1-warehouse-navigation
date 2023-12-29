import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'),'config')
    filename = 'my_rb1.lua'

    rviz_dir = os.path.join(get_package_share_directory('cartographer_slam'),'rviz')
    rviz_filename = 'mapper_config.rviz'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time':True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', filename]
    
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(rviz_dir, rviz_filename)]
    
    )


    return LaunchDescription([
        rviz_node,
        cartographer_node,
        occupancy_grid_node,
        
    ])