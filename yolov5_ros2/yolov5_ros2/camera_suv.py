import rclpy
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import time
import tf_transformations   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 
from happymini_teleop.base_control import BaseControl


class CameraSuv(Node):
    def __init__(self):
        super().__init__('camera_suv')
        self.sub = self.create_subscription(Float32MultiArray, 'topic', self.Callback, 10)
        self.xx = 0
        self.zz = 0
        self.dig = 0

    def Callback(self, msg):
        #self.get_logger().info(f'sub:{msg.data}')
        self.xx = msg.data[0]
        self.zz = msg.data[2]
        self.dig = (math.atan2(self.xx, self.zz))*180 / math.pi
        self.get_logger().info(f'{self.dig}')
        plt.plot(self.xx, self.zz, '.')
        plt.xlim(-1, 1)
        plt.ylim(0, 3)
        plt.pause(0.1)
        plt.clf()

def main():
    print('start')
    rclpy.init()
    node = CameraSuv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C pushed')
    rclpy.shutdown()
