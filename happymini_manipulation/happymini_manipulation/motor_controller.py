import math
import time
import threading
import numpy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        # Publisher
        self.joint_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        # Subscriber
        #self.create_subscription()
        # Value
        self.wrist_param = 30
        self.joint_angle_list = []
        self.gripper_close = False
        self.joint_names = [
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'gripper'
                ]
    
    def inverse_kinematics(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        l0 = 0.49
        l1 = 0.128
        l2 = 0.124
        l3 = 0.126
        x -= l3
        y -= l0

        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle - math.pi/2
            wrist_angle = -1*(shoulder_angle + elbow_angle) - math.pi/2
            # gripper
            if self.gripper_close:
                gripper = math.radians(55)
            else:
                gripper = math.radians(-90)
            angle_list = list(map(math.degrees, [math.radians(90), shoulder_angle, elbow_angle, wrist_angle, gripper]))
            return angle_list
        except ValueError:
            self.get_logger().info('Can not move arm.')
            return [numpy.nan]*5

    def publish_joint(self, joint_angle, execute_time=2):
        #ljoint_angle[3] += self.wrist_param
        # deg => rad
        print(joint_angle)
        joint_angle = list(map(math.radians, joint_angle))
        # メッセージの作成
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = list(map(float, joint_angle))
        msg.points[0].time_from_start = Duration(
                seconds=int(execute_time), nanoseconds=(execute_time-int(execute_time))*1e9).to_msg()
        # パブリッシュ
        self.get_logger().info('Publish joint.')
        self.joint_pub.publish(msg)
        time.sleep(execute_time + 0.5)

    def gripper(self, gripper_close):
        self.gripper_close = gripper_close
        if self.gripper_close:
            self.joint_angle_list[4] = 55
        else:
            self.joint_angle_list[4] = -90
        self.publish_joint(self.joint_angle_list, 2)

    def manipulation(self, coordinate):
        self.joint_angle_list = self.inverse_kinematics(coordinate)
        self.publish_joint(self.joint_angle_list, 2)

    def give(self):
        self.joint_angle_list = [90, 45, -45, -90, -90]
        self.publish_joint(self.joint_angle_list)

    def carry(self):
        self.joint_angle_list = [90, -40, 40, 10, -90]
        self.publish_joint(self.joint_angle_list)

    def start_up(self):
        self.joint_angle_list = [90, -90, 90, 0.0, -90]
        self.publish_joint(self.joint_angle_list)

    


def main():
    rclpy.init()
    jc_node = JointController()
    #jc_node.manipulation([0.3, 0.5])
    #jc_node.gripper(True)
    jc_node.start_up()
    #jc_node.give()
    #jc_node.manipulation([0.3, 0.38])
    #jc_node.joint_angle_list[3] = 43
    #jc_node.publish_joint(jc_node.joint_angle_list)
    #jc_node.carry()
    jc_node.destroy_node()
    rclpy.shutdown()
