import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    shelf_detect_file = 'shelf_detect_params.yaml'
    shelf_attach_file = 'shelf_attach_params.yaml'
    shelf_detach_file = 'shelf_detach_params.yaml'
    
    detect_dir = os.path.join(get_package_share_directory('rb1_autonomy'),'config',shelf_detect_file)
    attach_dir = os.path.join(get_package_share_directory('rb1_autonomy'),'config',shelf_attach_file)
    detach_dir = os.path.join(get_package_share_directory('rb1_autonomy'),'config',shelf_detach_file)
    
    remapping = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
    
    shelf_detection_server = Node(
        package='shelf_detect',
        executable='shelf_detect_server',
        name='shelf_detection_server',
        output='screen',
        parameters=[detect_dir],
        remappings=remapping
    
    )

    shelf_attach_server = Node(
        package='shelf_attach',
        executable='shelf_attach_server',
        name='shelf_attach_server',
        output='screen',
        parameters=[attach_dir],
        remappings=remapping
    )

    shelf_detach_server = Node(
        package='shelf_detach',
        executable='shelf_detach_server',
        name='shelf_detach_server',
        output='screen',
        parameters=[detach_dir],
        remappings=remapping
    )


    return LaunchDescription([
    
        shelf_detection_server,
        shelf_attach_server,
        shelf_detach_server
    
    ])

