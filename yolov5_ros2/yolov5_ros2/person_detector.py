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

from rclpy.qos import qos_profile_sensor_data


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.img = Image()

        self.service = self.create_service(
            DetectPerson, 'fmm_person_service/detect', self.detectPerson)
        

    def image_callback(self, image : Image) -> None:
        self.img = image

    def detectPerson(self, request : DetectPerson.Request, response : DetectPerson.Response) -> DetectPerson.Response:
        response.result = self.img
        return response

def main(args=None):
    rclpy.init(args=args)
    persondetect_node = PersonDetector()
    rclpy.spin(persondetect_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()