import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String
import logging
import pyrealsense2 as rs
import numpy as np

mp_hands = mp.solutions.hands.Hands(
    max_num_hands=1,
    #手の検出の信頼度のしきい値
    min_detection_confidence=0.1,
    min_tracking_confidence=0.8)
mp_drawing = mp.solutions.drawing_utils


class HandPosePublish(Node):
    def __init__(self):
        super().__init__('hand')
        self.pub = self.create_publisher(String, 'hand', 10)
        #add
        #トピック名: way
        self.way = self.create_publisher(String, '/way', 10)

        self.timer = self.create_timer(1, self.timer_callback)
        
        # Configure RealSense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hand position: x=%f, y=%f, z=%f, Hand pose: x=%f, y=%f, z=%f'
        #add
        msg_01 = String()
        msg_01.data = f'Way: d=%c'
        way = None

        hand_position = None
        hand_pose = None
        image = None
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            image = np.asanyarray(color_frame.get_data())
            image.flags.writeable = False
            results = mp_hands.process(image)
            image.flags.writeable = True

            if results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, results.multi_hand_landmarks[0], mp.solutions.hands.HAND_CONNECTIONS)

                for hand_landmarks in results.multi_hand_landmarks:
                    # Extract hand position and pose information from landmarks
                    hand_position = hand_landmarks.landmark[0] 
                    hand_pose = hand_landmarks.landmark[8] 
                    #add
                    if hand_position.x > hand_pose.x:
                        
                        d = 'left'
                    elif hand_position.x < hand_pose.x:
                        
                        d = 'right'
                    
                    else:
                        d = 'I cannot recognize'

                   
            cv2.imshow('Hand Pose Detection', image)
            cv2.waitKey(1)

        if hand_position and hand_pose:
            msg.data = msg.data % (hand_position.x, hand_position.y, hand_position.z, hand_pose.x, hand_pose.y, hand_pose.z)
            self.pub.publish(msg)
            self.get_logger().info(f'pub: {msg.data}')
            #add
            msg_01.data = (d)
            self.way.publish(msg_01)
            self.get_logger().info(f'way: {msg_01.data}')

    def destroy(self):
        self.pipeline.stop()
        super().destroy_node()


def detect_hand_pose(args=None):
    rclpy.init(args=args)
    node = HandPosePublish()
    rclpy.spin(node)
    node.destroy()



if __name__ == '__main__':
    detect_hand_pose()

