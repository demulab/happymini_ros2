import os
from glob import glob
from setuptools import setup

package_name = 'yolov5_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeffrey Too Chuan TAN',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='YOLOv5 applications with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = yolov5_ros2.detect:main',
            'detector = yolov5_ros2.detector:main',
            'object_detection = yolov5_ros2.object_detection:main',
            "object_detection_multi = yolov5_ros2.object_detection_multi:main",
            "dump_detection = yolov5_ros2.dump_detection_results:main",
            "dump_fisheye = yolov5_ros2.dump_fisheye:main",
            'object_detection_tf = yolov5_ros2.object_detection_tf:main',
            'object_detection_srv = yolov5_ros2.object_detection_srv:main',
            'empty_seat_finder = yolov5_ros2.empty_seat_finder:main',
            'person_detector = yolov5_ros2.person_detector:main',
            'camera_suv = yolov5_ros2.camera_suv:main',
            'happymini_sekkin = yolov5_ros2.happymini_sekkin',
            'person_in_area = yolov5_ros2.person_in_area:main'
        ],
    },
)
