import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    autonomy_node = Node(
    
        package='rb1_autonomy',
        executable='autonomy_node',
        name='autonomy_node',
        parameters=[{'location_file': os.path.join(get_package_share_directory('rb1_autonomy'), 'config', 'locations_backup.yaml')},
                        {'groot_file': os.path.join(get_package_share_directory('rb1_autonomy'), 'config', 'bt_test_groot2.xml')}],
        remappings=[('/cmd_vel','/diffbot_base_controller/cmd_vel_unstamped')]
    
    )

    destination_provider_node = Node(
        package='des_provider',
        executable='des_provider_node',
        output='screen'
    
    )

    return LaunchDescription([
        
        autonomy_node,
        destination_provider_node
    ])