import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
import time
from happymini_manipulation.motor_controller import JointController

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
        self.get_logger().info("panti")
        self.grasp_bag.wait_for_server()
        self.get_logger().info("tinti")

        goal_future = self.grasp_bag.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("ゴール拒否")
            return

        self.get_logger().info("ゴールおけまる")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"結果: {result}")

    def execute(self):
        time.sleep(1.0)
        while self.hand_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info("hand_pose is empty ...")
        time.sleep(0.5)
        self.get_logger().info("unti")
        self.get_logger().info(f"hand_pose >>> {self.hand_pose}")
        self.send_goal(self.hand_pose)
        #self.send_goal('right')
        self.get_logger().info("onti")


class Navigation(Node):
    def __init__(self):
        super().__init__('test_navigation')
        


class GiveBag(Node):
    def __init__(self):
        super().__init__('test_give_bag')
        self.jc_node = JointController()
        

    def execute(self):
        time.sleep(1.0)
        self.get_logger().info("give a bag")
        #self.send_goal()
        self.jc_node.manipulation([0.17, 0.4])
        time.sleep(5.0)
        self.jc_node.start_up()


def main():
    rclpy.init()
    tn = TestNode()
    gb = GiveBag()
    try:
        #tn.execute()
        gb.execute()
    except KeyboardInterrupt:
        pass
    finally:
        tn.destroy_node()
        rclpy.shutdown()
