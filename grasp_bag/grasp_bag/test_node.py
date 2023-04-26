import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_grasp_node')
        # Action
        self.grasp_bag = ActionClient(self, GraspBag, 'grasp_bag_server')
        # Topic
        self.create_subscription(String, 'way', self.hand_pose_callback, 10)
        # Value
        self.hand_pose = None

    def hand_pose_callback(self, receive_msg):
        self.hand_pose = receive_msg.data

    def send_goal(self, left_right):
        goal_msg = GraspBag.Goal()
        goal_msg.left_right = left_right
        goal_msg.coordinate = [0.25, 0.4]
        self.get_logger().info('panti')
        self.grasp_bag.wait_for_server()
        self.get_logger().info('tinti')
        self.grasp_bag.send_goal(goal_msg)
        self.get_logger().info('tunku')

    def execute(self):
        time.sleep(1.0)
        while self.hand_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info("hand_pose is empty ...")
        time.sleep(0.5)
        self.get_logger().info("hand_pose >>> {self.hand_pose}")
        self.send_goal(self.hand_pose)
        #self.send_goal('right')
        self.get_logger().info('onti')
    


def main():
    rclpy.init()
    tn = TestNode()
    try:
        tn.execute()
    except KeyboardInterrupt:
        pass
    finally:
        tn.destroy_node()
        rclpy.shutdown()
