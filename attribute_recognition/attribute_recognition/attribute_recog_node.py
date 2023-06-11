#!/usur/bin/env python3
# -*- coding: utf-8 -*-
# Python

import rclpy
from rclpy.node import Node
from happymini_msgs.srv import AttributeRecognition
from .infer_deepmar import AttributeRecognizer, AttributeInfo
from .unique_attribute_recognizer import UniqueAttributeRecognizer
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
        self.__attribute_recognizer = UniqueAttributeRecognizer()
        #self.__attribute_recognizer = AttributeRecognizer(weights_path=self.__resource_path+"deepmar")
        self.service = self.create_service(AttributeRecognition, "/fmm_attrib_service/recognize", self.recognizeAttribute)


    def recognizeAttribute(self, request, response):
        img = self.__bridge.imgmsg_to_cv2(request.input)    
        #result = self.__attribute_recognizer.recognizeAttributesNumpy(img)
        result = self.__attribute_recognizer.recognizeAttributes(img)
        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    attribute_recog_node = AttributeRecognizeNode()
    rclpy.spin(attribute_recog_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
