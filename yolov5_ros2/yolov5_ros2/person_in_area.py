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

from happymini_msgs.srv import PersonMulti, PersonInArea


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

        self.srv_personinmap = self.create_service(PersonInArea, 'srv_personinarea', self.get_person_map_pose)

    def map_callback(self, data : OccupancyGrid):
        self.mapinfo = data.info
        self.map = np.asarray([data.data]).reshape((self.mapinfo.height, self.mapinfo.width))
        
        print(self.map.shape)
        cv2.imwrite('/home/demulab/test_data/map.png', self.map)
        #print(self.map)
        #print(self.mapinfo)

    #def get_robot_map_pose(self, transform):
    #    x = transform.transform.translation.x
    #    y = transform.transform.translation.y

    #    map_x = self.mapinfo.origin.position.x
    #    map_y = self.mapinfo.origin.position.y

    #    robox = int((x - map_x) / self.mapinfo.resolution)
    #    roboy = int((y - map_y) / self.mapinfo.resolution)

    #    print('robo x', robox)
    #    print('robo y', roboy)
    #    return pxx, pxy 

    def set_person_multi(self):
        self.personmulti = self.personclient.send_request()
        
    def set_personmulti_client(self, client):
        self.personclient = client

    def set_transform(self):
            try:
                self.t = self.tf_buffer.lookup_transform(
                        "base_link",
                        "map",
                        rclpy.time.Time())
            except TransformException as e:
                print("no tf found")
 

    def get_person_map_pose(self, req, res): #, robotx, roboty):
        self.set_person_multi()
        euler = euler_from_quaternion((self.t.transform.rotation.x, self.t.transform.rotation.y, self.t.transform.rotation.z, self.t.transform.rotation.w))        
        x = self.t.transform.translation.x
        y = self.t.transform.translation.y

        map_x = self.mapinfo.origin.position.x
        map_y = self.mapinfo.origin.position.y

        robotx = int((x - map_x) / self.mapinfo.resolution)
        roboty = int((y - map_y) / self.mapinfo.resolution)

        print('robo x', robotx)
        print('robo y', roboty)

        #print('r :', math.degrees(euler[0]))
        #print('p :', math.degrees(euler[1]))
        print('y :', math.degrees(euler[2]))
        min_dist = 10000
        min_idx = 0
        for i in range(len(self.personmulti.poses)):
            angle = math.atan2(self.personmulti.poses[i].z, self.personmulti.poses[i].x) - math.pi/2
            dist = math.sqrt(self.personmulti.poses[i].x**2 + self.personmulti.poses[i].z**2)
            print('people :', math.degrees(angle))
            print('peoplemap :', math.degrees(euler[2] - angle))
            people_x = -dist * math.cos(euler[2]-angle) 
            people_y = dist * math.sin(euler[2]-angle)
            print('peplex :', people_x)
            print('peopley :', people_y)
            print('dest :', dist)
            map_x = self.mapinfo.origin.position.x
            map_y = self.mapinfo.origin.position.y
            print(self.mapinfo.origin.position)
            pxx = robotx + int((people_x - map_x) / self.mapinfo.resolution)
            pxy = roboty + int((people_y - map_y) / self.mapinfo.resolution)
            cv2.circle(self.map, (pxy, pxx), 10, 255, -1)
            cv2.imwrite('/home/demulab/test_data/map2.png', self.map)
            print('pxx :', pxx)
            print('pxy :', pxy)
            people_map_x = x + int((people_x - map_x))
            people_map_y = y + int((people_y - map_y))

            if pxx > self.mapinfo.width*0.9 or pxy > self.mapinfo.height*0.98 :
                print('people out')
            elif pxx < self.mapinfo.width*-0.02 or pxy < self.mapinfo.height*0.15:
                print('people out')
            else: 
                if min_dist > dist:
                    min_dist = dist
                    min_idx = i
        if len(self.personmulti.poses) > 0 :
            print(self.personmulti.poses[min_idx])
            img = CvBridge().imgmsg_to_cv2(self.personmulti.images[min_idx])
            cv2.imwrite('/home/demulab/test_data/people.png', img)
            res.result = True
            res.pose = self.personmulti.poses[min_idx]
            res.map_pose.x = people_map_x
            res.map_pose.y = people_map_y
            return res
        else :
            res.result = False
            return res



def main():                                                                                                                         
    rclpy.init()
    node = PersonArea()
    srv = PersonClient()
    node.set_personmulti_client(srv)
    try:
        while True:
            node.set_transform()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

