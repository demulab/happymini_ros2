import sys
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Vector3
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster
from happymini_msgs.srv import PersonMulti

from yolov5_ros2.detector import Detector, parse_opt
from rclpy.callback_groups import ReentrantCallbackGroup                                                                            
from rclpy.executors import MultiThreadedExecutor
import threading


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.callback_group = ReentrantCallbackGroup()

        self.service = self.create_service(
                PersonMulti, 'fmm/multi', self.person_callback)
        

        self.target_name = 'person'
        self.frame_id = 'target'

        self.detector = Detector(**args)

        self.bridge = CvBridge()

        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info')
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw')
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
       

    def person_callback(self, request, response):
        if self.img_color.shape[0:2] != self.img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        img_color = self.img_color.copy()
        img_color, result = self.detector.detect(img_color)

        
        #num = 0
        #ny = np.linspace(0,10,100)
        #nx = ny
        #n = []
        point = [0, 0, 0]
        minpoint = []
        target = None
        img_msg_emv = CvBridge().cv2_to_imgmsg(img_color, 'bgr8')
        response.environment_image = img_msg_emv
        print('yolo')
        for r in result:
            if r.name == self.target_name:
                target = r
                u1 = round(target.u1)
                u2 = round(target.u2)
                v1 = round(target.v1)
                v2 = round(target.v2)
                u = round((target.u1 + target.u2) / 2)
                v = round((target.v1 + target.v2) / 2)
                print('yolo2')
                depth = np.median(self.img_depth[v1:v2+1, u1:u2+1])
                ppl_img = img_color[v1:v2+1, u1:u2+1]
                img_msg = CvBridge().cv2_to_imgmsg(ppl_img, 'bgr8')
                area =  (u2 - u1) * (v2 - v1)
                img_h, img_w, _ = img_color.shape
                arearatio = float(area)/float(img_h * img_w)

                if arearatio >= 0.03:
                    response.images.append(img_msg)
                    print(ppl_img.shape)
                    print('yolo3')

                    #if depth != 0:
                    z = depth * 1e-3
                    #z = min(dep)
                    fx = self.k[0]
                    fy = self.k[4]
                    cx = self.k[2]
                    cy = self.k[5]
                    x = z / fx * (u - cx)
                    y = z / fy * (v - cy)
                    point[0] = x
                    point[1] = y
                    point[2] = z

                    ppl_point =Vector3()
                    ppl_point.x = x
                    ppl_point.y = y
                    ppl_point.z = z
                    if abs(z) < 3: 
                        response.poses.append(ppl_point)
                        response.tags.append(r.name)
                        response.likelihood.append(r.conf)

                #if target is not None:
                #    minpoint.append(np.array(point))
                #    #msg = Float32MultiArray()
                #    self.get_logger().info(f'{minpoint}')
                #    #msg.data = list(point)
                #    #self.pub.publish(msg)
                #    #self.get_logger().info(f'pub:{msg.data}')                
                #point[:3] = [0, 0, 0]

        #if target is not None:
        #    zz = np.argmin(minpoint, axis = 0)
        #    cc = zz[2]
        #    pp = minpoint[cc]
        #    print(type(pp))
        #    #self.get_logger().info(
        #    #       f'{pp}' )
        #    msg = Float32MultiArray()
        #    msg.data = list(pp)
        #img_depth *= 16

        #if target is not None:
        #    pt1 = (int(target.u1), int(target.v1))
        #    pt2 = (int(target.u2), int(target.v2))
        #    cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)
        #print(response)
        return response




    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            self.img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            self.img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return
        self.k = msg_info.k

        

def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))

    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
