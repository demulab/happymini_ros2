import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from happymini_teleop.base_control import BaseControl

class VoiceToRotate(Node):
    def __init__(self):
        super().__init__('voice_to_rotate_node')
        self.create_subscription(UInt16, 'respeaker/doa', self.respeaker_callback, 10)
        # Module
        self.bc_node = BaseControl()
        # Value
        self.sub_angle = 999
        self.custom_angle = 999

    def respeaker_callback(self, receive_msg):
        self.sub_angle = receive_msg.data - 90
        self.custom_angle = self.sub_angle
        if self.sub_angle > 180 and self.sub_angle < 270:
            self.custom_angle = (self.sub_angle - 270) - 90
        #self.get_logger().info(f"custom_angle: {self.custom_angle}")

    def get_angle(self, timeout=10):
        voice_angle = self.custom_angle
        start_time = time.time()
        while time.time() - start_time <= 10 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now_time = time.time() - start_time
            if abs(voice_angle - self.custom_angle) <= 5 and now_time > 5.0:
                break
            else:
                #start_time = time.time()
                voice_angle = self.custom_angle
            print(voice_angle)
        return voice_angle

    def rotate(self):
        move_angle = self.get_angle()
        self.get_logger().info(f"{move_angle}")
        self.bc_node.rotate_angle(move_angle, speed=0.2)

def main():
    rclpy.init()
    vtr = VoiceToRotate()
    try:
        vtr.rotate()
    except:
        pass
    vtr.destroy_node()
    rclpy.shutdown()

