import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    is_real_robot = False
    
    if is_real_robot:

        config_file = 'mapper_params_online_async_real.yaml'
        sim_time = False
    else:
        config_file = 'mapper_params_online_async.yaml'
        sim_time = True

    config_dir = os.path.join(get_package_share_directory('localization_server'),'config',config_file)


    start_async_slam_toolbox_node = Node(
        parameters=[
          config_dir,
          {'use_sim_time': sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    shelf_detection_server = Node(
        package='shelf_detect',
        executable='shelf_detect_server',
        name='shelf_detect_server',
        output='screen'
    
    )

    shelf_attach_server = Node(
        package='shelf_attach',
        executable='shelf_attach_server',
        name='shelf_attach_server',
        output='screen',
        parameters=[{'activate_elevator': False},{'attach_velocity': 0.2}]
    )

    shelf_detach_server = Node(
        package='shelf_detach',
        executable='shelf_detach_server',
        name='shelf_detach_server',
        output='screen',
        parameters=[{'detach_velocity': 0.2}]
    )

    ld = LaunchDescription()

    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(shelf_detection_server)
    ld.add_action(shelf_attach_server)
    ld.add_action(shelf_detach_server)

    return ld

