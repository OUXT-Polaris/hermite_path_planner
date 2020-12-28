# Copyright (c) 2020 OUXT Polaris
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    planner_concatenator_param_file = LaunchConfiguration(
        'planner_concatenator_param_dir',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config', 'planner_concatenator.yaml'))
    curve_planner_param_file = LaunchConfiguration(
        'curve_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config', 'curve_planner.yaml'))
    stop_planner_parm_file = LaunchConfiguration(
        'stop_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config', 'stop_planner.yaml')
    )
    obstacle_planner_param_file = LaunchConfiguration(
        'obstacle_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config', 'obstacle_planner.yaml')
    )
    velocity_planner_param_file = LaunchConfiguration(
        'velocity_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config', 'velocity_planner.yaml')
    )
    local_waypoint_server_param_file = LaunchConfiguration(
        'local_waypoint_server_param_file',
        default=os.path.join(
            get_package_share_directory('local_waypoint_server'),
            'config', 'local_waypoint_server.yaml')
    )
    hermite_path_planner_parm_file = LaunchConfiguration(
        'hermite_path_planner_parm_file',
        default=os.path.join(
            get_package_share_directory('hermite_path_planner'),
            'config', 'hermite_path_planner.yaml')
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_concatenator_param_file',
            default_value=planner_concatenator_param_file,
            description='planner concatenator paramters'),
        DeclareLaunchArgument(
            'curve_planner_param_file',
            default_value=curve_planner_param_file,
            description='curve planner parameters'
        ),
        DeclareLaunchArgument(
            'stop_planner_parm_file',
            default_value=stop_planner_parm_file,
            description='stop planner parameters'
        ),
        DeclareLaunchArgument(
            'obstacle_planner_param_file',
            default_value=obstacle_planner_param_file,
            description='obstacle planner parameters'
        ),
        DeclareLaunchArgument(
            'hermite_path_planner_parm_file',
            default_value=hermite_path_planner_parm_file,
            description='hremite path planner parameters'
        ),
        DeclareLaunchArgument(
            'velocity_planner_param_file',
            default_value=velocity_planner_param_file,
            description='velocity planner parameters'
        ),
        DeclareLaunchArgument(
            'local_waypoint_server_param_file',
            default_value=local_waypoint_server_param_file,
            description='local waypoint server parameters'
        ),
        Node(
            package='hermite_path_planner_bringup',
            executable='hermite_path_planner_bringup_node',
            parameters=[
                planner_concatenator_param_file,
                curve_planner_param_file,
                obstacle_planner_param_file,
                stop_planner_parm_file,
                velocity_planner_param_file,
                local_waypoint_server_param_file,
                hermite_path_planner_parm_file],
            output='screen'),
    ])
