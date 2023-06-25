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
import os
from ament_index_python.packages import get_package_share_directory



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

        area_path = os.path.join(
            get_package_share_directory('happymini_navigation'), 
            'maps',
            'arena.png')
        self.area_img = cv2.imread(area_path, 0)
        self.ppl_loc_viz_img = cv2.imread(area_path, 1)
        self.discovered_ppl = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
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

        robotx = -int( (x + map_x) / self.mapinfo.resolution) 
        roboty = self.mapinfo.height + int((y + map_y) / self.mapinfo.resolution)



        print("map height, map width", self.mapinfo.height, self.mapinfo.width)
        print(self.ppl_loc_viz_img.shape)
        print(y)
        print(map_y)

        print('robo x', robotx)
        print('robo y', roboty)

        #print('r :', math.degrees(euler[0]))
        #print('p :', math.degrees(euler[1]))
        #print('y :', math.degrees(euler[2]))
        min_dist = 10000
        min_idx = 0
        ppl_found = False
        print(f"numbers of ppl found : {len(self.personmulti.poses)}")

        is_dup = [False for i in range(len(self.personmulti.poses))]
        out_of_area = [False for i in range(len(self.personmulti.poses))]
        distance_from_robot = []

        for i in range(len(self.personmulti.poses)):
            angle = math.atan2(self.personmulti.poses[i].z, self.personmulti.poses[i].x) - math.pi/2
            dist = math.sqrt(self.personmulti.poses[i].x**2 + self.personmulti.poses[i].z**2)
            print('people :', math.degrees(angle))
            print('peoplemap :', math.degrees(euler[2] - angle))
            people_x = -dist * math.cos(euler[2]-angle) 
            people_y = -dist * math.sin(euler[2]-angle)
            #print('peplex :', people_x)
            #print('peopley :', people_y)
            #print('dest :', dist)
            map_x = self.mapinfo.origin.position.x
            map_y = self.mapinfo.origin.position.y
            print("x in map ", people_x)
            print("y in map", people_y)



            pxx = robotx - int(people_x  / self.mapinfo.resolution) -1
            pxy = roboty - int(people_y / self.mapinfo.resolution) -1
            #cv2.circle(self.map, (pxy, pxx), 10, 255, -1)
            #cv2.imwrite('/home/demulab/test_data/map2.png', self.map)
            print('pxx :', pxx)
            print('pxy :', pxy)
            people_map_x = x + people_x - map_x
            people_map_y = y + people_y - map_y

            ppl_out = False
            if pxx >= self.mapinfo.width or pxy >= self.mapinfo.height:
                print("people out from map")
                out_of_area[i] = True
            elif pxx < 0 or pxy < 0:
                print("people out from map")
                out_of_area[i] = True
            elif self.area_img[pxy, pxx] == 0:
                print("people out from area")
                out_of_area[i] = True
            else:
                print("amount of discoverd ppl : ", len(self.discovered_ppl))
                for j in range(len(self.discovered_ppl)):
                    diff = math.sqrt((people_map_x - self.discovered_ppl[j][0]) ** 2 + (people_map_y - self.discovered_ppl[j][1]) **2)
                    print("checking distance from discovered person : ", diff)
                    if diff < 0.25:
                        is_dup[i] = True
                        print("same ppl found, skipping")
                        break


            cv2.circle(self.ppl_loc_viz_img, (robotx, roboty), 5, (255, 0, 0), -1)
            if not is_dup[i]:
                cv2.circle(self.ppl_loc_viz_img, (pxx, pxy), 5, (0, 0, 255) if out_of_area[i] else (0, 255, 0), -1)
                cv2.imwrite('/home/demulab/test_data/ppl_loc.png', self.ppl_loc_viz_img)

            distance_from_robot.append(dist)
        
            #cv2.circle(self.ppl_loc_viz_img, (map_origin_x, map_origin_y), 10, (255, 255,0), -1)

            ppl_transform = TransformStamped()

            ppl_transform.header.frame_id = "map"
            ppl_transform.header.stamp = self.get_clock().now().to_msg()
            ppl_transform.child_frame_id = f"ppl{i}"
            ppl_transform.transform.translation.x = people_map_x
            ppl_transform.transform.translation.y = people_map_y 
            #ppl_transform.header.frame_id = "map"
            #ppl_transform.header.stamp = self.get_clock().now().to_msg()
            #ppl_transform.child_frame_id = f"robot"
            #ppl_transform.transform.translation.x = x
            #ppl_transform.transform.translation.y = y 

            self.tf_broadcaster.sendTransform(ppl_transform)
            #if pxx > self.mapinfo.width*0.9 or pxy > self.mapinfo.height*0.98 :
            #    print('people out')
            #elif pxx < self.mapinfo.width*-0.02 or pxy < self.mapinfo.height*0.15:
            #    print('people out')


        distance_list = sorted(range(len(distance_from_robot)), key=distance_from_robot.__getitem__)

        if len(self.personmulti.poses) > 0:

            for i in range(len(distance_list)):
                if not is_dup[distance_list[i]] and not out_of_area[distance_list[i]]:
                    print(self.personmulti.poses[min_idx])
                    img = CvBridge().imgmsg_to_cv2(self.personmulti.images[min_idx])
                    cv2.imwrite('/home/demulab/test_data/people.png', img)
                    res.result = True
                    res.pose = self.personmulti.poses[min_idx]
                    res.map_pose.x = people_map_x
                    res.map_pose.y = people_map_y
                    self.discovered_ppl.append((people_map_x, people_map_y))
                    break
            return res
        else:
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

