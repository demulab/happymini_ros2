import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
import tf_transformations
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
  Point, Quaternion, Twist
from happymini_msgs.srv import NaviCoord


class CoordNavi(Node):
    def __init__(self):
        super().__init__('coord_navi')
        # Action
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Value
        self.navigation_flg = False

    def set_pose(self, goal_pose):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
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
        elif future.result().status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted')
            self.navigation_flg = False
        else:
            self.navigation_flg = None

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
        goal = self.set_pose(srv_req.coordinate)
        send_goal_flg = self.send_goal(goal)
        while self.navigation_flg is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        srv_res.result = self.navigation_flg and send_goal_flg
        self.get_logger().info(f"srv_res >>> {srv_res.result}")
        return srv_res


class NaviCoordServer(Node):
    def __init__(self):
        super().__init__('navi_coord_server')
        # Node
        self.coord_navi = CoordNavi()
        # Service
        self.navi_coord_srv = self.create_service(NaviCoord, 'navi_coord_server', self.coord_navi.do_navigation)
        self.get_logger().info("Ready to set /navi_coord_srever")


def main():
    rclpy.init()
    navi_coord_server = NaviCoordServer()
    try:
        rclpy.spin(navi_coord_server)
    except KeyboardInterrupt:
        pass
    navi_coord_server.destroy_node()
    rclpy.shutdown()
