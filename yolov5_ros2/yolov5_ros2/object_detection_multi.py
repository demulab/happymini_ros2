import sys
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from happymini_msgs.msg import DetectionResult
from yolov5_ros2.detector import Detector, parse_opt


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.detector = Detector(**args)

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(DetectionResult, "/fmm/detection_results", 10)

        
    def image_callback(self, msg):
        try:
            img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        new_img = img0.copy()
        img, result = self.detector.detect(img0)

        detection_result = DetectionResult()
        
        detection_result.environment_image = msg

        cv2.imshow('result', img)
        cv2.waitKey(1)
        for i, r in enumerate(result):
            v1 = round(r.v1)
            v2 = round(r.v2)
            u1 = round(r.u1)
            u2 = round(r.u2)
            img_msg = self.bridge.cv2_to_imgmsg(new_img[v1:v2+1, u1:u2+1], "bgr8")
            detection_result.images.append(img_msg)
            detection_result.tags.append(r.name)
            detection_result.likelihood.append(r.conf)

            
            #self.get_logger().info(
            #    f'{i}: ({r.u1}, {r.v1}) ({r.u2}, {r.v2})' +
            #    f' {r.name}, {r.conf:.3f}')
        self.pub.publish(detection_result)
            

def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
