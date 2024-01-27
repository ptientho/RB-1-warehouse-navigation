import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config_file = 'mapper_params_online_async_real.yaml'
    config_dir = os.path.join(get_package_share_directory('localization_server'),'config',config_file)

    start_async_slam_toolbox_node = Node(
        parameters=[
          config_dir,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )


    shelf_detection_server = Node(
        package='shelf_detect',
        executable='shelf_detect_real_server',
        name='shelf_detection_real_server',
        output='screen',
        parameters=[{'frame': 'robot_cart_laser'},{'front_shelf_offset': 0.6}]
    
    )

    shelf_attach_server = Node(
        package='shelf_attach',
        executable='shelf_attach_server',
        name='shelf_attach_server',
        output='screen',
        parameters=[{'real_robot': True},{'activate_elevator': True},{'attach_velocity': 0.15},{'from_frame':'robot_base_footprint'},
                    {'to_frame':'robot_cart_laser'}, {'front_offset': 0.5}]
    )

    shelf_detach_server = Node(
        package='shelf_detach',
        executable='shelf_detach_server',
        name='shelf_detach_server',
        output='screen',
        parameters=[{'detach_velocity': 0.1}]
    )


    return LaunchDescription([
    
        start_async_slam_toolbox_node,
        shelf_detection_server,
        shelf_attach_server,
        shelf_detach_server
    
    ])

