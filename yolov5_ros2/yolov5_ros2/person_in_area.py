import sys
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
#from yolov5_ros2.detector import Detector, parse_opt
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class PersonArea(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            amcl_pose_qos)

    def map_callback(self, data : OccupancyGrid):
        self.map = data.data
        self.mapinfo = data.info
        print(self.map)
        print(self.mapinfo)

    def get_robot_map_pose(self, transform):
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        map_x = self.mapinfo.origin.position.x
        map_y = self.mapinfo.origin.position.y

        pxx = int((x - map_x) / self.mapinfo.resolution)
        pxy = int((y - map_y) / self.mapinfo.resolution)

        print(pxx)
        print(pxy)


def main():                                                                                                                         
    rclpy.init()
    node = PersonArea()
    try:
        while True:
            try:
                t = node.tf_buffer.lookup_transform(
                        "base_link",
                        "map",
                        rclpy.time.Time())
                print(t)
                node.get_robot_map_pose(t)
            except TransformException as e:
                print("no tf found")
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

