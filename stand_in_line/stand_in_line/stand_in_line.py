import sympy
import math
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from stand_in_line.detect_pose_mod import DetectPoseModule
from tf_transformations import quaternion_from_euler
import geometry_msgs
from happymini_msgs.action import StandInLine
from happymini_msgs.msg import PointsAndImages
from happymini_msgs.srv import NaviCoord
from happymini_teleop.base_control import BaseControl


class NaviCoordClient(Node):
    def __init__(self):
        super().__init__('navi_coord_client_node')
        # Client
        self.navi_coord = self.create_client(NaviCoord, 'navi_coord_server')
        while not self.navi_coord.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info("navi_coord_server is not here ...")
        self.navi_coord_req = NaviCoord.Request()

    def request_send(self, coordinate):
        self.navi_coord_req.coordinate = coordinate

        future = self.navi_coord.call_async(self.navi_coord_req)
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.result() is not None:
            return future.result().result
        else:
            self.get_logger().info("Service call failed")
            return False


class StandInLineServer(Node):
    def __init__(self):
        super().__init__('stand_in_line_node')
        # Action
        self.__action_server = ActionServer(self, StandInLine, 'stand_in_line_server', self.action_main)
        # Client
        self.navi_coord_client = NaviCoordClient()
        # Subscriber
        self.detect_sub = self.create_subscription(PointsAndImages, "detect_person", self.detection_callback, 10)
        # Module
        self.pose_mod = DetectPoseModule()
        self.bc_node = BaseControl()
        # Value
        self.person_info = None
        self.line_degree = None
        self.orientation = None
        self.line_eq = None
        self.inclination = None

        self.get_logger().info("Ready to set /stand_in_line_server")

    def detection_callback(self, receive_msg):
        self.person_info = receive_msg
        #self.image_and_coord_show(self.person_info)

    def get_min_coord(self, coord_list):
        people_coord_dict = {}
        # 座標がキーで深度が値
        for num, data in enumerate(coord_list.points):
            people_coord_dict[repr(data)] = [data.z]
        # 深度が最大の座標、最小の座標を求める
        try:
            min_coord = eval(min(people_coord_dict))
        except ValueError:
            return None
        #max_coord = eval(max(people_coord_dict))
        return min_coord

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
        min_person_num = 2
        if len(coord_list) < min_person_num:
            return False, None
        before_data = None
        # 人同士の距離を推定
        for data in coord_list:
            data.x = -1*data.x
            # 一人目の座標を格納
            if before_data is None:
                before_data = data
                continue
            # 人同士の距離を計算
            between_dist = math.sqrt((before_data.x - data.x)**2 + (before_data.z - data.z)**2)
            # 閾値以下だったら次の人をみる
            if between_dist >= 1.5:
                pass
            elif between_dist < 1.5:
                between_dist_list.append(between_dist)
                # 最初の人のためだけのif文
                if len(between_dist_list) == 1:
                    person_coords.append(before_data)
                # 深度を格納
                person_coords.append(data)
            else:
                return False, None
            before_data = data
        # 人の距離値が2つ未満だったら処理しない
        if len(between_dist_list) < min_person_num - 1:
            return False, None
        # 2点を通る直線の傾きを比較
        before_data = None
        inclination = None
        inclination_list = []
        person_point = 0  # 直線の式に使うだけ
        for data in coord_list:
            # 一人目の座標を格納
            if before_data is None:
                before_data = data
                continue
            # 傾き計算
            try:
                inclination = (data.z-before_data.z)/(data.x-before_data.x)
                inclination_list.append(inclination)
            except ZeroDivisionError:
                return False, None
        # 傾きの分散
        average = sum(inclination_list)/len(inclination_list)
        tmp = 0
        for data in inclination_list:
            tmp += (data - average)**2
        dispersion = tmp/len(inclination_list)
        # 閾値より分散が大きかったら列じゃない
        if dispersion > 0.05:
            return False, None
        # 最後の２人の直線の傾きを度数法にして格納
        self.line_degree = math.degrees(math.atan(inclination))
        self.inclination = inclination
        return True, person_coords
    
    def got_face(self, image_list):
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
            return None
        # ランドマーク取得
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
        return got_face_list

    def estimate_line(self):
        while not self.person_info and rclpy.ok():
            self.get_logger().warn("None topic from /detect_person ...")
            time.sleep(1.0)
        if self.person_info == PointsAndImages():
            return None
        # depthから列があるか判定
        depth_flg, people_coord = self.judg_from_depth(self.person_info.points)
        if not depth_flg:
            return None
        # 顔があるかどうか判定
        got_face_list = self.got_face(self.person_info.images)
        faces_rate = []
        if got_face_list is None:
            return None
        self.orientation = 'back_line'
        for got_face in got_face_list:
            if got_face:
                self.orientation = 'front_line'  # 一回でも顔が認識されたらfront_line
                faces_rate.append(1)
            else:
                faces_rate.append(0)
        # 顔があれば前向きの列
        #self.orientation = 'back_line'
        #if sum(faces_rate)/len(faces_rate) > 0.5:
        #    self.orientation = 'front_line'
        people_coord_dict = {}
        # 座標がキーで深度が値
        for num, data in enumerate(people_coord):
            people_coord_dict[repr(data)] = [data.z]
        # 深度が最大の座標、最小の座標を求める
        front_coord = eval(min(people_coord_dict))
        back_coord = eval(max(people_coord_dict))
        result = front_coord
        if self.orientation == 'front_line':
            result = back_coord
        self.get_logger().info(f"{self.orientation}")
        return result

    def wait_in_line(self):
        max_dist = 2.0
        cnt = 0
        before_min_coord = None
        find_flg = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            min_coord = self.get_min_coord(self.person_info)
            # before_min_coordがなかったらmin_coordを代入
            if before_min_coord is None:
                before_min_coord = min_coord
            # 
            if not min_coord and cnt < 50 and not find_flg:
                sub_cnt = 0
                search_range = 50
                while sub_cnt < 1 and not find_flg:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if min_coord:
                        find_flg = True
                        break
                    self.bc_node.rotate_angle(search_range, 0, speed=0.3, time_out=10)
                    rclpy.spin_once(self, timeout_sec=2.0)
                    if min_coord:
                        find_flg = True
                        break
                    self.bc_node.rotate_angle(-1*search_range*2, 0, speed=0.3, time_out=10)
                    rclpy.spin_once(self, timeout_sec=2.0)
                    if min_coord:
                        find_flg = True
                        break
                    self.bc_node.rotate_angle(search_range)
                    rclpy.spin_once(self, timeout_sec=2.0)
                    if min_coord:
                        find_flg = True
                        break
                    sub_cnt += 1
                    print('wait')
            # ROS2の座標に合わせる
            if not min_coord:
                return False
            min_coord.x = -1*min_coord.x
            if min_coord.z >= max_dist or not min_coord:
                break
            # 列が前に進んだら続く
            if before_min_coord.z < min_coord.z and min_coord.z < max_dist and cnt > 20:
                self.bc_node.rotate_angle(math.degrees(math.atan2(min_coord.x, min_coord.z)), 0, speed=0.3, time_out=10)
                self.bc_node.translate_dist(min_coord.z - before_min_coord.z - 0.1, speed=0.2)
                before_min_coord = min_coord
                cnt = 0
            cnt += 1
        return True

    def action_main(self, goal_handle):
        feedback_msg = StandInLine.Feedback()
        feedback_msg.state = "Start line detection"
        goal_handle.publish_feedback(feedback_msg)
        cnt = 0
        sum_cnt = 0
        before_coordinate = None
        while rclpy.ok():
            time.sleep(0.01)
            rclpy.spin_once(self, timeout_sec=0.1)
            # 端の人の座標をもってくる
            coordinate = self.estimate_line()
            if sum_cnt > 60:
                self.get_logger().info("None person")
                # Response
                goal_handle.succeed()
                result = StandInLine.Result()
                result.result = False
                self.get_logger().info(f"Result: {result}")
                return result
            # before_coordinateがなければcoordinateを代入
            if before_coordinate is None:
                before_coordinate = coordinate
                continue
            if cnt > 30:
                feedback_msg.state = "Line up"
                goal_handle.publish_feedback(feedback_msg)
                # 直線の式2つ目
                line_x_param = 0.9
                sympy.var("x, z")
                if self.orientation == 'front_line':
                    param_eq = sympy.Eq(x, coordinate.x-line_x_param)
                    roll = 0.0
                    pitch = 0.0
                    yaw = math.radians(self.line_degree+90)
                else:
                    line_x_param = line_x_param - 0.2
                    param_eq = sympy.Eq(x, coordinate.x+line_x_param)
                    roll = 0.0
                    pitch = 0.0
                    yaw = -1*math.radians(self.line_degree)
                # 直線の式を作成
                line_eq = sympy.Eq(z-coordinate.z, self.inclination*(x-coordinate.x))
                # 直線の式からゴールを算出
                x_z_coord = sympy.solve([line_eq, param_eq], [x, z])
                quaterion = quaternion_from_euler(roll, pitch, yaw, 'rxyz')
                coordinate = [float(x_z_coord[z]), float(x_z_coord[x]), quaterion[2], quaterion[3]]
                self.get_logger().info(f"{coordinate}")
                # Navigation
                navi_result = self.navi_coord_client.request_send(coordinate=coordinate)
                # 人の方を向く
                if navi_result:
                    min_coord = self.get_min_coord(self.person_info)
                    self.bc_node.rotate_angle(math.degrees(math.atan2(-1*min_coord.x, min_coord.z)), 0, speed=0.3, time_out=10)
                    # 人を待つ
                    feedback_msg.state = "wait in line"
                    goal_handle.publish_feedback(feedback_msg)
                    _ = self.wait_in_line()
                # Response
                goal_handle.succeed()
                result = StandInLine.Result()
                result.result = navi_result
                self.get_logger().info(f"Result: {result}")
                return result
            if coordinate:
                cnt += 1
                # 端の人をみる
                if abs(coordinate.x) > 0.3 and abs(before_coordinate.z - coordinate.z) > 0.4:
                    self.get_logger().info(f"{math.degrees(math.atan2(coordinate.x, coordinate.z))}")
                    self.bc_node.rotate_angle(math.degrees(math.atan2(coordinate.x, coordinate.z)), 0, speed=0.3, time_out=10)
                    before_coordinate = coordinate
            else:
                cnt = 0
            sum_cnt += 1

def main():
    rclpy.init()
    stand_line_node = StandInLineServer()
    time.sleep(1.0)
    try:
        rclpy.spin(stand_line_node)
    except KeyboardInterrupt:
        pass
    stand_line_node.destroy_node()
    rclpy.shutdown()
