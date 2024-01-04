import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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

    ld = LaunchDescription()

    ld.add_action(start_async_slam_toolbox_node)

    return ld
