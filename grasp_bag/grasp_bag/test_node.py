import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_grasp_node')
        # Action
        self.action_server = ActionServer(self, GraspBag, 'grasp_bag_server', self.execute_action)
        # Client
#        self.gbs_srv = self.create_client(BagLocalization, 'bag_localization_server')
#        while not self.gbs_srv.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info("grasp_bag_server is not here ...")
#        self.gbs_srv_req = BagLocalization.Request()
#
#    def gbs_srv_request_send(self):
#        self.gbs_srv_req.left_right = 'all'
#        self.gbs_srv_req.degree = 180
#        self.gbs_srv_req.graph = False
#        future = self.gbs_srv.call_async(self.gbs_srv_req)
#        while rclpy.ok():
#            rclpy.spin_once(self, timeout_sec=0.1)
#            if future.done():
#                if future.result() is not None:
#                    responce = future.result()
#                    self.get_logger().info(f"Receive responce: {responce}")
#                else:
#                    self.get_logger().info("Service call failed")
#                break

    def execute_action(self, goal_handle):
        self.get_logger().info("execute_action")
        time.sleep(1.0)

        feedback_msg = GraspBag.Feedback()
        feedback_msg.state = "メッセージを確認中"
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)

        goal1 = goal_handle.request.left_right
        goal2 = goal_handle.request.coordinate

        feedback_msg.state = "メッセージ確認完了"
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)

        self.get_logger().info("left_right: {goal1}, coordinate: {goal2}")
        time.sleep(1.0)

        goal_handle.succeed()
        result = GraspBag.Result()
        result.result = True
        return result


def main():
    rclpy.init()
    tn = TestNode()
    try:
        rclpy.spin(tn)
    except KeyboardInterrupt:
        pass
    finally:
        tn.destroy_node()
        rclpy.shutdown()
