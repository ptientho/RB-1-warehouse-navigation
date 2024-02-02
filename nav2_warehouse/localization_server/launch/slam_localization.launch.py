import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file = 'mapper_params_online_async.yaml'
    config_dir = os.path.join(get_package_share_directory('localization_server'),'config',config_file)

    start_async_slam_toolbox_node = Node(
        parameters=[
          config_dir,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
    
        start_async_slam_toolbox_node,
        
    ])

