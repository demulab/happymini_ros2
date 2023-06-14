import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='happymini_voice',
            executable='speech_to_text',
            output='screen'),

        Node(
            package='happymini_voice',
            executable='wav_play',
            output='screen'),

        ])
