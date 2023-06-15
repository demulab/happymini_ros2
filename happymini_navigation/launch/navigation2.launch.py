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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # turtlebot3_navigation2 launch path
    turtlebot3_navigation2_launch = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch')
    
    # map
    map_dir = LaunchConfiguration(
            'map',
            default=os.path.join(
                get_package_share_directory('happymini_navigation'),
                'maps',
                'demulab_230604.yaml'))

    # location yaml path
    location_dir = LaunchConfiguration(
            'locations',
            default=os.path.join(
                get_package_share_directory('happymini_navigation'), 
                'location',
                'receptionist_230614.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value=map_dir,
            description='Path to map file to load'),

        DeclareLaunchArgument(
            name='locations',
            default_value=location_dir,
            description='Path to the location params YAML file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_navigation2_launch + '/navigation2.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='happymini_navigation',
            executable='navi_location',
            arguments=[location_dir],
            output='screen'),
        ])
