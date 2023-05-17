import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('grasp_bag'),
            'config',
            'grasp_bag_params.yaml')

    return LaunchDescription([
        Node(
            package='grasp_bag',
            executable='bag_localization_server',
            parameters=[config],
            output='screen'),
        Node(
            package='grasp_bag',
            executable='grasp_bag_server', 
            output='screen',
            on_exit=launch.actions.Shutdown()),
        ])
