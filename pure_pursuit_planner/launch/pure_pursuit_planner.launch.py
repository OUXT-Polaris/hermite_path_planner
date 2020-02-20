import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration

def generate_launch_description():
    description = LaunchDescription([
        Node(
            package='pure_pursuit_planner',
            node_executable='pure_pursuit_planner_node',
            node_name='pure_pursuit_planner_node',
            output='screen'),
    ])
    return description