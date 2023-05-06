import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String
import logging
import pyrealsense2 as rs
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
    super().__init__('pose')
    self.pub = self.create_publisher(String, 'hand', 10)

    self.way = self.create_publisher(String, '/way', 10)

    self.timer = self.create_timer(1, self.timer_callback)

    self.pipeline = rs.pipeline()
    self.config = rs.config()
    self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)

    self.pipeline.start(self.config)

  def timer_callback(self):
    msg = String()
    msg.data = f'shoulder_right: x=%f, y=%f, z=%f, shoulder_left: x=%f, y=%f, z=%f, hand_right: x=%f, y=%f, z=%f, hand_left: x=%f, y=%f, z=%f'
    
    msg_01 = String()
    msg_01.data = f'way: d=%c'

    shoulder_right = None
    shoulder_left = None
    hand_right = None
    hand_left = None
    image = None

    frames = self.pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if color_frame:
      image = np.asanyarray(color_frame.get_data())
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
          

          if (shoulder_right.x - hand_right.x) > 0.1:
            way = 'left'
          elif (hand_left.x - shoulder_left.x) > 0.1:
            way = 'right'
          else:
            way = 'all'
      
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

def detect_hand_pose(args=None):
  rclpy.init(args=args)
  node = HandPosePublish()
  rclpy.spin(node)
  node.destroy()

if __name__=='__main__':
  detect_hand_pose()
