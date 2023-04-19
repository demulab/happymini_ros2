import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class OutputOdom(Node):
    def __init__(self):
        super().__init__('output_odom_node')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, receive_msg):
        quaternion = (
                receive_msg.pose.pose.orientation.x,
                receive_msg.pose.pose.orientation.y,
                receive_msg.pose.pose.orientation.z,
                receive_msg.pose.pose.orientation.w)
        current_deg = math.degrees(euler_from_quaternion(quaternion)[2])
        if current_deg < 0.0:
            current_deg = 180 + (180 - abs(current_deg))
        print(current_deg)
        time.sleep(0.1)


def main():
    rclpy.init()
    oo = OutputOdom()
    try:
        rclpy.spin(oo)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
