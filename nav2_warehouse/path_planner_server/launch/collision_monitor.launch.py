#!/usr/bin/env python3

# Copyright (c) 2022 Samsung R&D Institute Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Constant parameters
    lifecycle_nodes = ['collision_monitor']
    autostart = True

    # Launch arguments
    # 1. Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    
    params_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_robot', 'collision.yaml')

    # 2. Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': True},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_collision_monitor_cmd = Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[params_file])

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_namespace_cmd)

    # Node launching commands
    ld.add_action(start_collision_monitor_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    

    return ld