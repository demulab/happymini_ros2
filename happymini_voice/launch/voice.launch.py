import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('happymini_voice'),
            'config',
            'pyttsx3_property.yaml')

    return LaunchDescription([
        Node(
            package='happymini_voice',
            executable='text_to_speech',
            parameters=[config],
            output='screen'),
        ])
