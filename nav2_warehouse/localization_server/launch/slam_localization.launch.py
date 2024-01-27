import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config_file = 'mapper_params_online_async.yaml'
    config_dir = os.path.join(get_package_share_directory('localization_server'),'config',config_file)

    remapping = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
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


    shelf_detection_server = Node(
        package='shelf_detect',
        executable='shelf_detect_server',
        name='shelf_detection_server',
        output='screen',
        remappings=remapping
    
    )

    shelf_attach_server = Node(
        package='shelf_attach',
        executable='shelf_attach_server',
        name='shelf_attach_server',
        output='screen',
        parameters=[{'activate_elevator': False}],
        remappings=remapping
    )

    shelf_detach_server = Node(
        package='shelf_detach',
        executable='shelf_detach_server',
        name='shelf_detach_server',
        output='screen',
        parameters=[{'detach_velocity': 0.1}],
        remappings=remapping
    )


    return LaunchDescription([
    
        start_async_slam_toolbox_node,
        shelf_detection_server,
        shelf_attach_server,
        shelf_detach_server
    
    ])

