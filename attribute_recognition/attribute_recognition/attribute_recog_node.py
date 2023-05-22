#!/usur/bin/env python3
# -*- coding: utf-8 -*-
# Python

import rclpy
from rclpy.node import Node
from .infer_deepmar import AttributeRecognizer, AttributeInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import time
from ament_index_python import get_package_share_directory

class AttributeRecognizeNode(Node):
    def __init__(self):
        super().__init__("attribute_recog_node")
        self.__bridge = CvBridge()
        self.__resource_path =str(get_package_share_directory("attribute_recognition"))
        print(self.__resource_path)
        self.__resource_path += "/../ament_index/resource_index/packages/"
        self.__attribute_recognizer = AttributeRecognizer(weights_path=self.__resource_path+"deepmar")
        self.__pub = self.create_publisher(String, "/findmymates/attribute_sentence", 10)
        self.__sub = self.create_subscription(Image, "/image_raw", self.cb, 10)


    def cb(self, msg: Image):
        img = self.__bridge.imgmsg_to_cv2(msg)
        start = time.time()
        result = self.__attribute_recognizer.recognizeAttributesNumpy(img)
        end = time.time()
        print(result)
        print("elapsed time : {0}s".format(end - start))
        msg = String()
        msg.data = result
        self.__pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    attribute_recog_node = AttributeRecognizeNode()
    rclpy.spin(attribute_recog_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
