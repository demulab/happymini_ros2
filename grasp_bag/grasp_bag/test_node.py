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
        self.grasp_bag.wait_for_server()
        self._send_goal_future = self.grasp_bag.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("ゴールが拒否されました")
            return
        self.get_logger().info("ゴールが承認されました")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"結果：{result}")

    def goal_response_callback(self, future):
        goal_handle = future.result()

    def feedback_callback(self, feedback_msg):
        #feedback = feedback_msg.state
        self.get_logger().info(feedback_msg.feedback.state)

    def execute(self):
        time.sleep(1.0)
        #while self.hand_pose is None and rclpy.ok():
        #    rclpy.spin_once(self, timeout_sec=0.5)
        #    self.get_logger().info("hand_pose is empty ...")
        #time.sleep(0.5)
        #self.get_logger().info("hand_pose >>> {self.hand_pose}")
        #self.send_goal(self.hand_pose)
        self.send_goal('right')
        print("Finish")
        #self.send_goal('right')
    


def main():
    rclpy.init()
    tn = TestNode()
    try:
        tn.execute()
        rclpy.spin(tn)
    except KeyboardInterrupt:
        pass
    finally:
        tn.destroy_node()
        rclpy.shutdown()
