import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grasp_bag', executable='grasp_bag_server', 
            output='screen', on_exit=launch.actions.Shutdown()),
        Node(
            package='grasp_bag', executable='bag_localization_node', 
            output='screen', name='bag_localization_node')
        ])
