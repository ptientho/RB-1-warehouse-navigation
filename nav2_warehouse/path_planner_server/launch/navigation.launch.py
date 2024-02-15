from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    # include localization
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('localization_server'), 'launch'), '/localization_sim.launch.py']))

    # include planner
    planner = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('path_planner_server'), 'launch'), '/path_planner.launch.py']))

    return LaunchDescription([
        localization,
        planner
    ])