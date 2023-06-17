import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
from happymini_manipulation.motor_controller import JointController
from happymini_teleop.base_control import BaseControl


class BagLocalizationClient(Node):
    def __init__(self):
        super().__init__('bag_localization_client')
        # Client
        self.bl_srv = self.create_client(BagLocalization, 'bag_localization_server')
        while not self.bl_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("bag_localization_server is not here ...")
        self.bl_srv_req = BagLocalization.Request()

    def request_send(self, left_right, scan_range=100, graph=False):
        bl_srv_result = {}
        self.bl_srv_req.left_right = left_right
        self.bl_srv_req.degree = scan_range
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

class ParamsGetClient(Node):
    def __init__(self):
        super().__init__('param_get_node')
        self.params_name = ['estimation.scan_range', 'estimation.dispersion']
        self.params_client = self.create_client(GetParameters, '/bag_localization_node/get_parameters')
        while not self.params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Param get server not available ...')

    def get_params(self):
        # Value
        params_dict = {}
        values_list = []
        # Send Request
        req = GetParameters.Request()
        req.names = self.params_name
        future = self.params_client.call_async(req)
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.result() is not None:
            values_list = future.result().values
            index = 0
            for data in values_list:
                param = data.integer_value
                params_dict[self.params_name[index]] = param
                self.get_logger().info(f'{params_dict}')
                index += 1
            return params_dict
        else:
            self.get_logger().info(f"Could not get params ...")
            return None


class GraspBagServer(Node):
    def __init__(self):
        super().__init__('grasp_bag_node')
        # Action
        self._action_server = ActionServer(self, GraspBag, 'grasp_bag_server', self.execute)
        # Client
        self.blc_node = BagLocalizationClient()
        self.pgc_node = ParamsGetClient()
        # Module
        self.bc_node = BaseControl()
        self.jc_node = JointController()
        self.get_logger().info("Ready to set /grasp_bag_server")

    def execute(self, goal_handle):
        move_angle = 3
        left_right = goal_handle.request.left_right
        self.get_logger().info("Executing grasp_bag_action_server ...")
        self.jc_node.start_up()
        # 1段階目
        feedback_msg = GraspBag.Feedback()
        feedback_msg.state = "Estimating bag location ..."
        goal_handle.publish_feedback(feedback_msg)
        params = self.pgc_node.get_params()
        self.get_logger().info(f"Params: {params}")
        bag_info = self.blc_node.request_send(left_right, scan_range=params['estimation.scan_range'])
        self.get_logger().info(f"Bag info >>> {bag_info}")
        feedback_msg.state = "Approaching a bag"
        goal_handle.publish_feedback(feedback_msg)
        self.bc_node.rotate_angle(bag_info['angle_to_bag'], precision=0, speed=0.2, time_out=20)
        time.sleep(1.0)
        self.bc_node.translate_dist((bag_info['distance_to_bag'] - 0.20)/2, 0.1)
        # 2段階目
        feedback_msg.state = "Re: Estimating bag location ..."
        goal_handle.publish_feedback(feedback_msg)
        bag_info = self.blc_node.request_send('all', 120)
        self.get_logger().info(f"Bag info >>> {bag_info}")
        self.bc_node.rotate_angle(bag_info['angle_to_bag'], precision=0, speed=0.2, time_out=10)
        #self.bc_node.translate_dist((bag_info['distance_to_bag'] - 0.1)/2, 0.05)
        if left_right == 'left':
            move_angle = -1*move_angle
        time.sleep(0.5)
        self.bc_node.rotate_angle(move_angle, precision=0, speed=0.2, time_out=10)
        time.sleep(1.0)
        # 把持
        feedback_msg.state = "Grasp a bag"
        goal_handle.publish_feedback(feedback_msg)
        self.jc_node.manipulation([0.2, 0.38])
        self.jc_node.manipulation([0.3, 0.38])
        self.jc_node.gripper(True)
        self.jc_node.joint_angle_list[3] = 43
        self.jc_node.publish_joint(self.jc_node.joint_angle_list)
        time.sleep(0.5)
        self.jc_node.carry()
        # レスポンス
        goal_handle.succeed()
        result = GraspBag.Result()
        result.result = True
        self.get_logger().info(f"Result: {result}")
        return result


def main():
    rclpy.init()
    gbs = GraspBagServer()
    try:
        gbs.jc_node.start_up()
        rclpy.spin(gbs)
    except KeyboardInterrupt:
        pass
    gbs.destroy_node()
    rclpy.shutdown()
