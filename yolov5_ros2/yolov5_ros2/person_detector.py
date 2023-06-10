import sys
import time
import threading
import queue
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from happymini_msgs.srv import DetectPerson
from yolov5_ros2.detector import Detector, parse_opt

from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError

class PersonDetector(Node):
    def __init__(self, **args):
        super().__init__("person_detector")
        self.detector = Detector(**args)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.img = Image()
        self.bridge = CvBridge()
        self.service = self.create_service(
            DetectPerson, 'fmm_person_service/detect', self.detectPerson)
        

    def image_callback(self, image : Image) -> None:
        self.img = image

    def detectPerson(self, request : DetectPerson.Request, response : DetectPerson.Response) -> DetectPerson.Response:
        try:
            img0 = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        new_img = img0.copy()
        img, result = self.detector.detect(img0)
        
        likelihood = 0
        ppl_img = np.ndarray([])
        for i, r in enumerate(result):
            if r.name == "person" and r.conf > likelihood:
                v1 = round(r.v1)
                v2 = round(r.v2)
                u1 = round(r.u1)
                u2 = round(r.u2)
                ppl_img = new_img[v1:v2+1, u1:u2+1]
                likelihood = r.conf
        
        h, w, _ = ppl_img.shape
        if h*w > 0:
            img_msg = self.bridge.cv2_to_imgmsg(ppl_img, "bgr8")
        else:
            img_msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")

        response.result = img_msg

        return response

def main(args=None):
    rclpy.init(args=args)
    persondetect_node = PersonDetector()
    rclpy.spin(persondetect_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
