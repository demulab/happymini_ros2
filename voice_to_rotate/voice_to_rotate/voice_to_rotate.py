import time
import statistics
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

    def get_angle(self, timeout=10):
        angle_list = []
        voice_angle = 0
        start_time = time.time()
        self.get_logger().info('Now getting audio ...')
        while time.time() - start_time <= timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # 誤差5度未満だったらリストに格納
            if abs(voice_angle - self.custom_angle) <= 5:
                angle_list.append(voice_angle)
            else:
                voice_angle = self.custom_angle
            # 要素数が10以上になったらbreak
            if len(angle_list) >= 30:
                break
        result_angle = statistics.mode(angle_list)  # 最頻値
        return voice_angle

    def rotate(self):
        move_angle = self.get_angle()
        self.get_logger().info(f"{move_angle}")
        self.bc_node.rotate_angle(move_angle, speed=0.2)

def main():
    rclpy.init()
    vtr = VoiceToRotate()
    try:
        while rclpy.ok():
            vtr.rotate()
            time.sleep(1.0)
    except:
        pass
    vtr.destroy_node()
    rclpy.shutdown()

