import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    LOCATION_FILE_NAME = 'test_navi_map'

    # turtlebot3_navigation2
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    read_launch = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_navigation2_dir + '/launch/navigation2.launch.py'
                ])
            )
    
    # location param
    #yaml_file = LOCATION_FILE_NAME + '.yaml'
    #yaml_path = os.path.join(
    #        get_package_share_directory('happymini_navigation'), 
    #        'location',
    #        yaml_file)

    return LaunchDescription([
        #DeclareLaunchArgument(
        #    name='yaml_path',
        #    default_value=yaml_path,
        #    description='Full path to the location params YAML file'),

        read_launch,

        #Node(
        #    package='happymini_navigation',
        #    executable='navi_location',
        #    arguments=[LaunchConfiguration('yaml_path')],
        #    output='screen',),

        #Node(package='happymini_navigation',
        #     executable='navi_location',
        #     output='screen',)
        ])
