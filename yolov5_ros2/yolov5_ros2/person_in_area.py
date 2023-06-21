import sys
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
#from yolov5_ros2.detector import Detector, parse_opt
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from happymini_msgs.srv import PersonMulti


class PersonClient(Node):
    def __init__(self):
        super().__init__('person_client')
        self.person_srv = self.create_client(PersonMulti, 'fmm/multi')
        self.person_req = PersonMulti.Request()

    def send_request(self):
        stt_srv_result = 'None'
        #self.peson_req.cmd = cmd
        stt_srv_future = self.person_srv.call_async(self.person_req)
        while not stt_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if stt_srv_future.result() is not None:
            stt_srv_result = stt_srv_future.result()
            #print(stt_srv_result)
            return stt_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

class PersonArea(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            amcl_pose_qos)

    def map_callback(self, data : OccupancyGrid):
        self.map = data.data
        self.mapinfo = data.info
        #print(self.map)
        #print(self.mapinfo)

    def get_robot_map_pose(self, transform):
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        map_x = self.mapinfo.origin.position.x
        map_y = self.mapinfo.origin.position.y

        pxx = int((x - map_x) / self.mapinfo.resolution)
        pxy = int((y - map_y) / self.mapinfo.resolution)

        #print(pxx)
        #print(pxy)
        return pxx, pxy 

    def get_person_map_pose(self, personmulti, t, robotx, roboty):
        euler = euler_from_quaternion((t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w))
        #print('r :', math.degrees(euler[0]))
        #print('p :', math.degrees(euler[1]))
        print('y :', math.degrees(euler[2]))
        for i in range(len(personmulti.poses)):
            angle = math.atan2(personmulti.poses[i].z, personmulti.poses[i].x) - math.pi/2
            dist = math.sqrt(personmulti.poses[i].x**2 + personmulti.poses[i].z**2)
            print('people :', math.degrees(angle))
            print('peoplemap :', math.degrees(euler[2] - angle))
            people_x = dist * math.cos(angle) 
            people_y = dist * math.sin(angle)

            map_x = self.mapinfo.origin.position.x
            map_y = self.mapinfo.origin.position.y
        
            pxx = robotx + int((people_x - map_x) / self.mapinfo.resolution)
            pxy = roboty + int((people_y - map_y) / self.mapinfo.resolution)
            print('pxx :', pxx)
            print('pxy :', pxy)

            if pxx > self.mapinfo.width*0.9 or pxy > self.mapinfo.height*0.9 :
                print('people out')
            elif pxx < self.mapinfo.width*0.1 or pxy < self.mapinfo.height*0.1:
                print('people out')
            else: 
                print('people in')




def main():                                                                                                                         
    rclpy.init()
    node = PersonArea()
    srv = PersonClient()
    try:
        while True:
            try:
                t = node.tf_buffer.lookup_transform(
                        "base_link",
                        "map",
                        rclpy.time.Time())
                #print(t)
                robotx,roboty = node.get_robot_map_pose(t)
                personmulti = srv.send_request()
                node.get_person_map_pose(personmulti, t, robotx, roboty)
            except TransformException as e:
                print("no tf found")
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

