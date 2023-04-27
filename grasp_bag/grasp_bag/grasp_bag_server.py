import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
#from happymini_manipulation.motor_controller import JointController
from happymini_teleop.base_control import BaseControl


class GraspBagServer(Node):
    def __init__(self):
        super().__init__('grasp_bag_server_node')
        # Action
        self._action_server = ActionServer(self, GraspBag, 'grasp_bag_server', self.execute)
        # Client
        self.bl_srv = self.create_client(BagLocalization, 'bag_localization_server')
        while not self.bl_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("bag_localization_server is not here ...")
        self.bl_srv_req = BagLocalization.Request()
        # Timer
        #self.feedback_timer = self.create_timer(0.1, self.feedback_publish)
        # Module
        self.bc_node = BaseControl()
        #self.jc_node = JointController()

        self.get_logger().info("Ready to set grasp_bag_server")

    def bl_srv_request_send(self, left_right, graph=False):
        bl_srv_result = {}
        self.bl_srv_req.left_right = left_right
        self.bl_srv_req.degree = 100
        self.bl_srv_req.graph = graph
        bl_srv_future = self.bl_srv.call_async(self.bl_srv_req)
        while not bl_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if bl_srv_future.result() is not None:
            bl_srv_result['angle_to_bag'] = bl_srv_future.result().angle_to_bag
            bl_srv_result['distance_to_bag'] = bl_srv_future.result().distance_to_bag
            return bl_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

    def execute(self, goal_handle):
        move_angle = 6
        feedback_msg = GraspBag.Feedback()
        #self.jc_node.start_up()
        # 1段階目
        feedback_msg.state = "Estimating bag location ..."
        goal_handle.publish_feedback(feedback_msg)
        bag_info = self.bl_srv_request_send(goal_handle.request.left_right)
        self.get_logger().info(f"Bag info >>> {bag_info}")
        feedback_msg.state = f"Bag info >>> {bag_info}"
        goal_handle.publish_feedback(feedback_msg)
        feedback_msg.state = "Approaching bag ..."
        goal_handle.publish_feedback(feedback_msg)
        self.bc_node.rotate_angle(bag_info['angle_to_bag'])
        time.sleep(1.0)
        self.bc_node.translate_dist((bag_info['distance_to_bag'] - 0.35)/2, 0.1)
        # 2段階目
        feedback_msg.state = "Re: Estimating bag location ..."
        goal_handle.publish_feedback(feedback_msg)
        bag_info = self.bl_srv_request_send('all', True)
        self.get_logger().info(f"Bag info >>> {bag_info}")
        feedback_msg.state = f"Bag info >>> {bag_info}"
        goal_handle.publish_feedback(feedback_msg)
        self.bc_node.rotate_angle(bag_info['angle_to_bag'])
        if goal_handle.request.left_right == 'left':
            move_angle = -1*move_angle
        time.sleep(0.5)
        self.bc_node.rotate_angle(move_angle)
        time.sleep(1.0)
        # 把持
        #self.jc_node.manipulation([0.2, 0.45])
        #self.jc_node.manipulation([0.3, 0.45])
        #self.jc_node.manipulation([0.3, 0.5])
        
        #self.bc_node.translate_dist((bag_info['distance_to_bag'] - 0.4)/2, 0.05)
        time.sleep(0.5)
        #self.jc_node.start_up()
        # レスポンス
        goal_handle.succeed()
        result = GraspBag.Result()
        result.result = True
        return result


def main():
    rclpy.init()
    gbs = GraspBagServer()
    try:
        rclpy.spin(gbs)
    except KeyboardInterrupt:
        pass
    gbs.destroy_node()
    rclpy.shutdown()
