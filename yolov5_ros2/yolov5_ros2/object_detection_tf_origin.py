import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Point, PoseArray
from std_msgs.msg import ByteMultiArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

from yolov5_ros2.detector import Detector, parse_opt


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.target_name = 'person'
        self.frame_id = 'target'

        self.detector = Detector(**args)

        self.bridge = CvBridge()
        
        self.img_depth = None
        # camera
        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info')
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw')
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        self.broadcaster = TransformBroadcaster(self)
        # by kanazawa
        self.coord_pub = self.create_publisher(ByteMultiArray, 'person_coordinates', 10)
        self.person_coord = Point()
        self.coordinates = ByteMultiArray()

    def generation_coord(self, msg_info, data):
        u1 = round(data.u1)
        u2 = round(data.u2)
        v1 = round(data.v1)
        v2 = round(data.v2)
        u = round((data.u1 + data.u2) / 2)
        v = round((data.v1 + data.v2) / 2)
        depth = np.median(self.img_depth[v1:v2+1, u1:u2+1])
        if depth != 0:
            z = depth * 1e-3
            fx = msg_info.k[0]
            fy = msg_info.k[4]
            cx = msg_info.k[2]
            cy = msg_info.k[5]
            x = z / fx * (u - cx)
            y = z / fy * (v - cy)
            self.person_coord.x = x
            self.person_coord.y = y
            self.person_coord.z = z

    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            self.img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != self.img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        img_color, result = self.detector.detect(img_color)
        cv2.imshow('color', img_color)

        target = None
        max_num = 5
        person_list = []
        for r in result:
            if len(person_list) > max_num:
                break
            elif r.name == self.target_name:
                person_list.append(r)

        if person_list:
            self.coordinates.data.clear()
            for data in person_list:
                self.generation_coord(msg_info, data)
                self.coordinates.data.append(self.person_coord)
        print(self.coordinates)
        #self.coord_pub.publish(self.coordinates)

        self.img_depth *= 16
        if target is not None:
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(self.img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        cv2.imshow('depth', self.img_depth)
        cv2.waitKey(1)


def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
