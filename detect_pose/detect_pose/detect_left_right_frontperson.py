import time
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Point
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster
#from std_msgs.msg import Image
#import logging
#import pyrealsense2 as rs
import numpy as np
from happymini_msgs.msg import PointsAndImages




mp_drawing_styles = mp.solutions.drawing_styles

mp_pose = mp.solutions.pose.Pose(
  min_detection_confidence=0.1,
  min_tracking_confidence=0.8
)
mp_pose_01= mp.solutions.pose

mp_drawing = mp.solutions.drawing_utils

class HandPosePublish(Node):
    def __init__(self):
        super().__init__('detect_left_right')
        self.pub = self.create_publisher(String, 'hand', 10)  
        self.way = self.create_publisher(String, '/way', 10)
        self.detect_sub = self.create_subscription(PointsAndImages, 'detect_person', self.detection_callback, 10)
        # Value
        self.person_info = None
  
    def detection_callback(self, receive_msg):
        self.person_info = receive_msg
        if not self.person_info.images:
            self.get_logger().warn("None image")
            time.sleep(1.0)
        else:
            # 最小値の人を取得
            before_point = None
            # 画像のデータが１より多かったら一番近い人を取得する
            if len(self.person_info.images) > 1:
                for num, point in enumerate(self.person_info.points):
                    if before_point is None:
                        before_point = point
                        continue
                    imgmsg = self.person_info.images[num]
                    if point.z > before_point.z:
                        imgmsg = self.person_info.images[num - 1]
            else:
                imgmsg = self.person_info.images[0]
            try:
                img = CvBridge().imgmsg_to_cv2(imgmsg)
            except CvBridgeError as e:
                self.get_logger().warn(str(e))
            msg = String()
            msg.data = f'shoulder_right: x=%f, y=%f, z=%f, shoulder_left: x=%f, y=%f, z=%f, hand_right: x=%f, y=%f, z=%f, hand_left: x=%f, y=%f, z=%f'
            
            msg_01 = String()
            msg_01.data = f'way: d=%c'
  
            shoulder_right = None
            shoulder_left = None
            hand_right = None
            hand_left = None
            image = None
  
            color_frame = img
            if color_frame.any():
              image = np.asanyarray(color_frame)
              #self.get_logger().info(f'{image}')
              image.flags.writeable = False
              #image = cv2.cvtColor(image, cv2.COLOE_RGB2BGR)
              results = mp_pose.process(image)
              image.flags.writeable = True
              if results.pose_landmarks:
                #mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose_01.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                for pose_landmarks in results.pose_landmarks.landmark:
                  
                  shoulder_left = results.pose_landmarks.landmark[11]
                  shoulder_right = results.pose_landmarks.landmark[12]
                  hand_left = results.pose_landmarks.landmark[15]
                  hand_right = results.pose_landmarks.landmark[16]
                  
                  right = abs(shoulder_right.x - hand_right.x)
                  left = abs(hand_left.x - shoulder_left.x)
                  if right - left > 0:
                    way = 'left'
                  elif left - right > 0:
                    way = 'right'
                  else:
                    way = 'None pose'
              
              cv2.imshow('Pose', image)
              cv2.waitKey(1)
  
            if shoulder_right and shoulder_left and hand_right and hand_left:
              msg.data = msg.data % (shoulder_right.x, shoulder_right.y, shoulder_right.z, shoulder_left.x, shoulder_left.y, shoulder_left.z, hand_right.x, hand_right.y, hand_right.z, hand_left.x, hand_left.y, hand_left.z)
              self.pub.publish(msg)
              self.get_logger().info(f'pub: {msg.data}')
              msg_01.data = (way)
              self.way.publish(msg_01)
              self.get_logger().info(f'way: {msg_01.data}')
    
    def destroy(self):
        self.pipeline.stop()
        super().destroy_node

def main(args=None):
  rclpy.init(args=args)
  node = HandPosePublish()
  rclpy.spin(node)
  node.destroy()
