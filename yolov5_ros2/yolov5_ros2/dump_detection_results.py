import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from happymini_msgs.msg import DetectionResult
from cv_bridge import CvBridge

"""
A Node for dumping object detection results for debugging
"""
class DumpDetectionResults(Node):
    def __init__(self):
        super().__init__("dump_detection")

        self.subscription = self.create_subscription(
            DetectionResult,
            "/fmm/detection_results",
            self.detection_callback,
            qos_profile_sensor_data
        )
        self.counter = 0
        self.bridge = CvBridge()

        
    def detection_callback(self, msg : DetectionResult):

        max_like = 0
        for i in range(len(msg.tags)):
            if msg.tags[i] == "person" and msg.likelihood[i] > max_like:
                max_like = msg.likelihood[i]
                img = self.bridge.imgmsg_to_cv2(msg.images[i], "bgr8")

        self.counter += 1
        cv2.imwrite("/home/demulab/data/{0}.png".format(self.counter), img)

def main():
    rclpy.init()
    node = DumpDetectionResults()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
