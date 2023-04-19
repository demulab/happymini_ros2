import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
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
        # Module
        self.bc_node = BaseControl()

        self.get_logger().info("Ready to set grasp_bag_server")

    def bl_srv_request_send(self, left_right):
        bl_srv_result = {}
        self.bl_srv_req.left_right = left_right
        self.bl_srv_req.degree = 180
        self.bl_srv_req.graph = False
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
        self.get_logger().info("Executing grasp_bag_action_server ...")
        bag_info = self.bl_srv_request_send(goal_handle.request.left_right)
        self.get_logger().info(f"Bag info >>> {bag_info}")
        self.bc_node.rotate_angle(bag_info['angle_to_bag'])
        time.sleep(1.0)
        self.bc_node.translate_dist((bag_info['distance_to_bag'] - 0.3)/2, 0.1)
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
