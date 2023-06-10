from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='happymini_voice',
            namespace='happymini_voice',
            executable='speech_to_text',
        ),
        Node(
            package='happymini_voice',
            namespace='happymini_voice',
            executable='name_detect',
        ), 
        Node(
            package='attribute_recognition',
            namespace='attribute_recognition',
            executable='attribute_recog_node',
        ),
        Node(
            package='yolov5_ros2',
            namespace='yolov5_ros2',
            executable='object_detection_tf',
        ), 
        Node(
            package='yolov5_ros2',
            namespace='yolov5_ros2',
            executable='person_detector',
        ), 
        Node(
            package='approach_person',
            namespace='approach_person',
            executable='approach_person',
        ),
    ])
