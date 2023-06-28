#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# Python
import yaml
import sys, time
from math import pi
import os
from ament_index_python.packages import get_package_share_directory
# ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
  Point, Quaternion, Twist
import tf_transformations
from tf_transformations import euler_from_quaternion
from nav2_msgs.action import NavigateToPose
# Custom
from happymini_msgs.srv import NaviLocation


class WayPointNavi(Node):
    def __init__(self):
        super().__init__('waypoint_navi')
        # Action
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # YAML
        self.yaml_path = sys.argv[1]
        self.get_logger().info(f"Load the following YAML file: {self.yaml_path}")
        # Value
        self.navigation_flg = False
        self.location_name = None
        self.location_dict = {}
        self.param_namespace = 'location_params'
        self.set_params()

    def set_params(self):
        # Open yaml
        with open(self.yaml_path) as f:
            self.location_dict = yaml.safe_load(f)
        self.get_logger().info(f"'{self.location_dict}' already set")
        # Set param
        #location_params = [[key, value] for key, value in self.location_dict.items()]
        #self.declare_parameters(self.param_namespace, location_params)
        ## Check param
        #print(self.location_dict)
        #for key in self.location_dict.keys():
        #    param_name = self.param_namespace + '.' + key
        #    if self.has_parameter(param_name):
        #        self.get_logger().info(f"'{param_name}' already set")
        #    else:
        #        self.get_logger().error(f"Could not set '{param_name}'")
        #        break 

    def search_location_param(self, location_name):
        #param_name = self.param_namespace + '.' + location_name
        #param_name = self.location_dict[location_name]
        #if self.has_parameter(param_name):
        #location_coordinate = self.get_parameter(param_name).value
        location_coordinate = self.location_dict[location_name]
        self.get_logger().info(f"{location_name}: {location_coordinate}")
        return location_coordinate
        #else:
        #    self.get_logger().error(f"'{location_name}' doesn't exist.'")
        #    return None

    def set_pose(self, goal_pose):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = goal_pose[0]
        pose.pose.position.y = goal_pose[1]
        pose.pose.orientation.z = goal_pose[2]
        pose.pose.orientation.w = goal_pose[3]
        return pose

    def send_goal(self, pose): 
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Wait for action server ...")
        self.get_logger().info("Start Navigation")
        # goal msg
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(
                          goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=3.0)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("Rejected !!!")
            return False
        
        self.get_logger().info("Approved !")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
        return True

    def get_result_callback(self, future): 
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation success !')
            self.navigation_flg = True
        elif future.result().status == GoalStatus.ABORTED:
            self.get_logger().error('Navigation aborted')
            self.navigation_flg = False

    def feedback_callback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().info(f"To Goal >>> {self.feedback.distance_remaining:.2f} [m]")

    def cancel_goal(self):
        self.get_logger().info('Cancel No.{}．')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)

    def do_navigation(self, srv_req, srv_res):
        self.navigation_flg = None
        send_goal_flg = True
        location_coordinate = self.search_location_param(srv_req.location_name)
        if location_coordinate:
            goal = self.set_pose(location_coordinate)
            send_goal_flg = self.send_goal(goal)
        while self.navigation_flg is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        srv_res.result = self.navigation_flg and send_goal_flg
        self.get_logger().info(f"srv_res >>> {srv_res.result}")
        return srv_res


class NaviLocationServer(Node):
    def __init__(self):
        super().__init__('navi_location_server')
        # Node
        self.wp_node = WayPointNavi()
        # Service
        self.navi_location_srv = self.create_service(NaviLocation, 'navi_location_server', self.wp_node.do_navigation)
        self.get_logger().info("Ready to set /navi_location_srever")


def main(args=None):
    rclpy.init(args=args)
    navi_location_server = NaviLocationServer()
    try:
        rclpy.spin(navi_location_server)
    except KeyboardInterrupt:
        pass
    #navi_location_server.destroy_node()
    rclpy.shutdown()
