#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('the_construct_office_gazebo')

    # Add to path the models for Ingestors and Digestors
    extra_models_package = "the_construct_office_gazebo"
    extra_models_package_path = get_package_share_directory(extra_models_package)
    extra_models_path = os.path.join(extra_models_package_path, 'models')

    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(gazebo_plugins_name)


    ####################################
    # Plugins
    gazebo_ros2_control_name = "gazebo_ros2_control"
    gazebo_ros2_control_path_install_dir = get_package_prefix(
        gazebo_ros2_control_name)
    ####################################


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + extra_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  extra_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + gazebo_plugins_name_path_install_dir + '/lib' + ':' + gazebo_ros2_control_path_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugins_name_path_install_dir + '/lib' + ':' + gazebo_ros2_control_path_install_dir + '/lib'

    #print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    #print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_bot_gazebo, 'worlds', 'ros2_online_workshop.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Set "true" to increase messages written to terminal.'
        ),
        gazebo
    ])