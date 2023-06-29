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
        #self.timer = self.create_timer(0.05, self.timer_callback)
        # pipe
        #self.pipeline = rs.pipeline()
        #self.config = rs.config()
        #self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
        #self.pipeline.start(self.config)
        # realsense
        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info')
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw')
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        self.broadcaster = TransformBroadcaster(self)
  
    def image_callback(self, msg_info, msg_color, msg_depth):
        try:
            img = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().info(str(e))
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
              
  
              if (shoulder_right.x - hand_right.x) > 0.05:
                way = 'left'
              elif (hand_left.x - shoulder_left.x) > 0.05:
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
