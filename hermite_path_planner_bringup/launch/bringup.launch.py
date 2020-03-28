import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    planner_concatenator_param_file = LaunchConfiguration(
        'planner_concatenator_param_dir',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','planner_concatenator.yaml'))
    curve_planner_param_file = LaunchConfiguration(
        'curve_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','curve_planner.yaml'))
    stop_planner_parm_file = LaunchConfiguration(
        'stop_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','stop_planner.yaml')
    )
    obstacle_planner_param_file = LaunchConfiguration(
        'obstacle_planner_param_file',
        default=os.path.join(
            get_package_share_directory('velocity_planner'),
            'config','obstacle_planner.yaml')
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
        Node(
            package='hermite_path_planner_bringup',
            node_executable='hermite_path_planner_bringup_node',
            parameters=[
                planner_concatenator_param_file,
                curve_planner_param_file,
                obstacle_planner_param_file,
                stop_planner_parm_file],
            output='screen'),
    ])