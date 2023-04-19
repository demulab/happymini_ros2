import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String
import logging
import pyrealsense2 as rs
import numpy as np


class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscriber = self.create_subscription(String, 'way', self.timer_callback, 10)

    def timer_callback(self,msg_01):
        if msg_01.data == 'right':
            self.get_logger().info('subscriber:go to right')
        elif msg_01.data == 'left':
            self.get_logger().info('subscriber:go to left')
        else:
            self.get_logger().info('I do not know')

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    node = subscriber

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()