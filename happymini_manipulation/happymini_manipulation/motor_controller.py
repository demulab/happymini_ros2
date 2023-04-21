import math
import time
import threading
import numpy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        self.joint_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = [
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'gripper'
                ]
        self.joint_angle_list = []
    
    def deg_to_rad(self, deg):
        return math.radians(deg)

    def inverse_kinematics(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        l0 = 0.55
        l1 = 0.128
        l2 = 0.124
        l3 = 0.126
        x -= l3
        y -= l0

        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle - math.pi/2
            wrist_angle = -1*(shoulder_angle + elbow_angle) - math.pi/2
            angle_list = list(map(math.degrees, [0.0, shoulder_angle, elbow_angle, wrist_angle, -90]))
            print(angle_list)
            return angle_list
        except ValueError:
            self.get_logger().info('Can not move arm.')
            return [numpy.nan]*5

    def publish_joint(self, joint_angle, execute_time=2):
        self.get_logger().info('Publish joint.')
        joint_angle = list(map(self.deg_to_rad, joint_angle))
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = list(map(float, joint_angle))
        msg.points[0].time_from_start = Duration(
                seconds=int(execute_time), nanoseconds=(execute_time-int(execute_time))*1e9).to_msg()
        self.joint_pub.publish(msg)
        time.sleep(2.5)

    def manipulation(self, coordinate):
        self.joint_angle_list = self.inverse_kinematics(coordinate)
        self.publish_joint(self.joint_angle_list, 2)

    def gripper(self, gripper_close):
        if gripper_close:
            self.joint_angle_list[4] = 55
        else:
            self.joint_angle_list[4] = -90
        self.publish_joint(self.joint_angle_list, 2)

    def start_up(self):
        self.joint_angle_list = [0.0, -90, 90, 0.0, -90]
        self.publish_joint(self.joint_angle_list)
    


def main():
    rclpy.init()
    jc_node = JointController()
    #jc_node.start_up()
    jc_node.manipulation([0.2, 0.7])
    #jc_node.manipulation([0.3, 0.4])
    #jc_node.gripper(False)
    #jc_node.manipulation([0.3, 0.45])
    #jc_node.gripper(True)
    #jc_node.start_up()
    jc_node.destroy_node()
    rclpy.shutdown()
