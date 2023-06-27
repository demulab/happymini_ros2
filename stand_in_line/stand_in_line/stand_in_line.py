import math
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from happymini_msgs.msg import PointsAndImages
from stand_in_line.detect_pose_mod import DetectPoseModule


class StandInLineServer(Node):
    def __init__(self):
        super().__init__('stand_in_line_node')
        # Subscriber
        self.detect_sub = self.create_subscription(PointsAndImages, "detect_person", self.detection_callback, 10)
        # Module
        self.pose_mod = DetectPoseModule()
        # Value
        self.person_info = None

    def detection_callback(self, receive_msg):
        self.person_info = receive_msg
        #self.image_and_coord_show(self.person_info)

    def image_and_coord_show(self, person_info):
        # images
        for num, img in enumerate(person_info.images):
            try:
                img = CvBridge().imgmsg_to_cv2(img)
            except CvBridgeError as e:
                self.get_logger().warn(str(e))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imshow(f"Image number {num}", img)
            cv2.waitKey(1)
        # coordinates
        for data in person_info.points:
            plt.plot(data.x, data.z, '.')
        plt.xlim(-5, 5)
        plt.ylim(0, 7)
        plt.pause(0.1)
        plt.clf()

    def judg_from_depth(self, coord_list):
        between_dist_list = []
        person_coords = []
        # 3人未満だったら処理しない
        if len(coord_list) < 3:
            return False, None
        before_data = None
        # 人同士の距離を推定
        for data in coord_list:
            # 一人目の座標を格納
            if before_data is None:
                before_data = data
                continue
            # 人同士の距離を計算
            between_dist = math.sqrt((before_data.x - data.x)**2 + (before_data.z - data.z)**2)
            # 閾値以下だったら次の人をみる
            if between_dist >= 1.5:
                pass
            elif between_dist < 1.5
                between_dist_list.append(between_dist)
                # 最初の人のためだけのif文
                if len(between_dist_list) == 1:
                    person_coords.append(before_data.z)
                # 深度を格納
                person_coords.append(data.z)
            else:
                return False, None
            before_data = data
        # 人の距離値が2つ未満だったら処理しない
        if len(between_dist_list) < 2:
            return False, None
        # 2点を通る直線の傾きを比較
        before_data = None
        inclination = None
        inclination_list = []
        for data in coor_list:
            # 一人目の座標を格納
            if before_data is None:
                before_data = data
                continue
            # 傾き計算
            inclination = (data.z-before_data.z)/(data.x-before_data.x)
            inclination_list.append(inclination)
        # 傾きの分散
        average = sum(inclination_list)/len(inclination_list)
        for data in inclination_list:
            tmp += (data - average)**2
        dispersion = tmp/len(inclination_list)
        # 閾値より分散が大きかったら列じゃない
        if dispersion > 0.5:
            return False, None
        return True, person_coords

    def judg_from_pose(self, image_list):
        images = []
        orientation_list = []
        # 率
        side_rate = []
        got_face_rate = []
        back_rate = []
        orientation_rate = []
        try:
            for img in image_list:
                img = CvBridge().imgmsg_to_cv2(img)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                images.append(img)
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return False
        # ランドマーク検出
        pose_list, got_face_list = self.pose_mod.detect(images=images)
        before_orientation = 0
        orientation = 0
        for num, pose in enumerate(pose_list):
            try:
                face_sign = numpy.nan
                shoulder_sign = numpy.nan
                hip_sign = numpy.nan
                face_orientation = 999.9
                shoulder_orientation = 999.9
                hip_orientation = 999.9
                body_orientation = numpy.nan
                upper_orientation = numpy.nan
                # sign
                face_sign = pose['left_ear'].x - pose['right_ear'].x
                shoulder_sign = pose['left_shoulder'].x - pose['right_shoulder'].x
                hip_sign = pose['left_hip'].x - pose['right_hip'].x
                # orientation
                face_orientation = pose['left_ear'].x/pose['right_ear'].x
                shoulder_orientation = pose['left_shoulder'].x/pose['right_shoulder'].x
                hip_orientation = pose['left_hip'].x/pose['right_hip'].x
                # body (None face) or upper
                body_orientation = (shoulder_orientation+hip_orientation)/2
                upper_orientation = (face_orientation+shoulder_orientation+hip_orientation)/3
            except KeyError as key:
                pass
            # 人の向きを推定
            # 背中
            if shoulder_sign < 0 and hip_sign < 0:
                if before_orientation is None:
                    before_orientation = 'back'
                    orientation_list.append(before_orientation)
                    continue
                orientation = 'back'
            # 正面
            elif shoulder_sign > 0 and hip_sign > 0:
                if before_orientation is None:
                    before_orientation = 'front'
                    orientation_list.append(before_orientation)
                    continue
                orientation = 'front'
            # わからない
            else:
                if before_orientation is None:
                    before_orientation = 'None'
                    orientation_list.append(before_orientation)
                    continue
                orientation = 'None'
            # 前の人と向きが同じか
            if before_orientation == orientation:
                orientation_rate.append(1)
                before_orientation = orientation
            else:
                orientation_rate.append(0)
            # 向きを格納
            orientation_list.append(orientation)
        if sum(orientation_rate)/len(orientation_rate) < 0.5:
            return False
        orientation_result = 'back'
        if len(orientation_list.count('front'))/len(orientation_list) > 0.5:
            orientation_result = 'front'
        return True, orientation_result

    def tmp_main(self):
        while not self.person_info and rclpy.ok():
            self.get_logger().warn("None topic from /detect_person ...")
            time.sleep(1.0)
        if self.person_info == PointsAndImages():
            return None
        depth_flg, people_depth = self.judg_from_depth(self.person_info.points)
        pose_flg, orientation = self.judg_from_pose(self.person_info.images)
        if not depth_flg and pose_flg:
            return None
        if orientation == 'front':
            max(people_depth)

        return None


def main():
    rclpy.init()
    stand_line_node = StandInLineServer()
    time.sleep(1.0)
    try:
        #rclpy.spin(stand_line_node)
        while rclpy.ok():
            rclpy.spin_once(stand_line_node, timeout_sec=0.1)
            _ = stand_line_node.tmp_main()
    except KeyboardInterrupt:
        pass
    stand_line_node.destroy_node()
    rclpy.shutdown()
