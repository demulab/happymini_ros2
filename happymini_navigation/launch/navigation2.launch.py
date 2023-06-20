import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # turtlebot3_navigation2 launch file
    turtlebot3_navigation2_launch = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch')
    
    # location param
    yaml_path = os.path.join(
            get_package_share_directory('happymini_navigation'), 
            'location',
            'demulab3.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='yaml',
            default_value=yaml_path,
            description='Full path to the location params YAML file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_navigation2_launch + '/navigation2.launch.py'])
        ),

        #Node(
        #    package='happymini_navigation',
        #    executable='navi_location',
        #    arguments=[yaml_path],
        #    output='screen'),
        ])
