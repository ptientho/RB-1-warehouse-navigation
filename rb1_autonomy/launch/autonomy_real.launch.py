import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    autonomy_node = Node(
    
        package='rb1_autonomy',
        executable='autonomy_node',
        name='autonomy_node',
        parameters=[{'location_file': os.path.join(get_package_share_directory('rb1_autonomy'), 'config', 'locations.yaml')},
                    {'real_robot': True}, {'tree_node_xml_file':'bt_test_groot2_real.xml'}]
    
    )

    return LaunchDescription([
        
        autonomy_node
    ])