import time
import rclpy
from rclpy.node import Node
import time
from happymini_manipulation.motor_controller import JointController

class tamesi(Node):
    def __init__(self):
        super().__init__('tamesi')
        self.jc_node = JointController()

    def execute(self):
        self.jc_node.start_up()
        self.jc_node.manipulation([0.2, 0.5])
        #self.jc_node.manipulation([0.3, 0.38])
        #self.jc_node.joint_angle_list[3] = 43
        self.jc_node.publish_joint(self.jc_node.joint_angle_list)
        self.jc_node.start_up()

def main():
    rclpy.init()
    try:
        tamesi().execute()
    except KeyboardInterrupt:
        tamesi().destroy_node()
        rclpy.shutdown()
