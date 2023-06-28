import sys
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from happymini_msgs.srv import DetectEmptyChair, TrackPerson
from yolov5_ros2.detector import Detector, parse_opt
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class EmptySeatFinder(Node):

    def __init__(self, **args):
        super().__init__('empty_seat_finder')

        self.detector = Detector(**args)

        self.bridge = CvBridge()
        chair_cb_group = MutuallyExclusiveCallbackGroup()
        person_cb_group = MutuallyExclusiveCallbackGroup()
        image_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data, callback_group=image_group)
        self.service = self.create_service(
            DetectEmptyChair, 'recp/find_seat', self.detectEmptyChair, callback_group=chair_cb_group)

        self.service = self.create_service(
            TrackPerson, 'recp/track_person', self.trackPerson, callback_group= person_cb_group
        )

        self.is_tracked = False
        self.angle = 0.0
        self.cnt = 0

    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return
        
    def calculateAreaRatio(self, target: dict, w: int, h: int) -> float:
        x = (target["u2"] - target["u1"])/2
        y = (target["v2"] - target["v1"])/2
        return float(x*y)/float(w*h)

    def trackPerson(self, request : TrackPerson.Request, response: TrackPerson.Response) -> TrackPerson.Response:
        response.angle = 0.0
        new_img = self.img.copy()
        img, result = self.detector.detect(new_img)
        cv2.imwrite("/home/demulab/test_data/detect.png", img)
        img_h, img_w, _ = img.shape
        print("w, h : {0}, {1}".format(img_w, img_h))
        
        people = []
        
        for i, r in enumerate(result):
            print(r.name)
            if r.name == "person":
                print("people found")
                people_info = dict()
                people_info["v1"] = round(r.v1)
                people_info["v2"] = round(r.v2)
                people_info["u1"] = round(r.u1)
                people_info["u2"] = round(r.u2)
                people.append(people_info)

        
        print("{0} people are detected".format(len(people)))
        people_angles = []
        #calculate yaw angle 
        for i in range(len(people)):
            yaw = self.calculateYawAngle(people[i], img_w, 180)
            people_angles.append(yaw)


        print("people detected in : {0}".format(people_angles))

        if len(people_angles) == 0:
            self.is_tracked = False
            response.angle = self.angle
            return response
        
        if self.is_tracked is False:
            self.is_tracked = True
            maxarea = 0
            max_idx = 0
            print(people)
            # nearest people as tracking target
            for i in range(len(people)):
                area = self.calculateAreaRatio(people[i], img_w, img_h)
                if area > maxarea:
                    max_idx = i
                    maxarea = area
            if len(people) > 0:
                response.angle = people_angles[max_idx]
            else:
                self.is_tracked = False
                response.angle = self.angle
        else:
            min_diff = 360
            min_idx = 0
            # find nearest person from the last target 
            for i in range(len(people)):
                diff = abs(self.angle - people_angles[i])
                if diff < min_diff:
                    min_idx = i
                    min_diff = diff
            if len(people) > 0:
                response.angle = people_angles[min_idx]
            else:
                self.is_tracked = False
                response.angle = self.angle
        self.angle = response.angle

        return response

    
    def detectEmptyChair(self, request : DetectEmptyChair.Request, response : DetectEmptyChair.Response) -> DetectEmptyChair.Response:
        self.cnt += 1
        new_img = self.img.copy()
        img, result = self.detector.detect(new_img)
        cv2.imwrite("/home/demulab/test_data/detect.png", img)
        img_h, img_w, _ = img.shape
        print("w, h : {0}, {1}".format(img_w, img_h))
        
        chair = []
        people = []

        for i, r in enumerate(result):
            if r.name == "chair":
                chair_info = dict()
                chair_info["v1"] = round(r.v1)
                chair_info["v2"] = round(r.v2)
                chair_info["u1"] = round(r.u1)
                chair_info["u2"] = round(r.u2)
                if self.calculateAreaRatio(chair_info, img_w, img_h) > 0.005:
                    print("chair is in the near range")
                    chair.append(chair_info)
                else:
                    print("removing far chairs")
                    print("area : {0}".format(self.calculateAreaRatio(chair_info, img_w, img_h)))
            if r.name == "people":
                people_info = dict()
                people_info["v1"] = round(r.v1)
                people_info["v2"] = round(r.v2)
                people_info["u1"] = round(r.u1)
                people_info["u2"] = round(r.u2)
                people.append(people_info)

        
        full_seats = []


        #remove full seats from IOU parameter
        for i in range(len(chair)):
            for j in range(len(people)):
                if self.calculateIOU(chair[i], people[j]) >= 0.8:
                    full_seats.append(i)
                    break

        empty_seats = []

        # store empty seats
        for i in range(len(chair)):
            if i in full_seats:
                continue
            else:
                empty_seats.append(i)

        max_area = 0
        max_idx =0

        for i in range(len(chair)):
            chair_h = chair[empty_seats[i]]["v2"] - chair[empty_seats[i]]["v1"]
            chair_w = chair[empty_seats[i]]["u2"] - chair[empty_seats[i]]["u1"]
            area = chair_h * chair_w
            if max_area < area:
                max_area = area
                max_idx = i

        #calculate yaw angle 
        #for i in range(len(empty_seats)):
        #    yaw = self.calculateYawAngle(chair[empty_seats[i]], img_w, 180)
        #    response.angles.append(yaw)


        report_img = self.img.copy()
        if len(empty_seats) >0: 
            yaw = self.calculateYawAngle(chair[empty_seats[max_idx]], img_w, 180)
            response.angles.append(yaw)
            print(chair[empty_seats[max_idx]])
            cv2.rectangle(report_img, (chair[empty_seats[max_idx]]["u1"], chair[empty_seats[max_idx]]["v1"]), (chair[empty_seats[max_idx]]["u2"], chair[empty_seats[max_idx]]["v2"]), (0,255,0), 3)
            cv2.imwrite(f"/home/demulab/test_data/recp_chair{self.cnt}.png", report_img)
        return response

        
        


    def calculateYawAngle(self, target : dict, width : int, fov : int) -> float:
        center = width/2
        pix_angle = float(fov/width)

        x = int((target["u1"] + target["u2"])/2)
        yaw = float(x - center) * pix_angle
        return yaw


    def calculateIOU(self, A : dict, B :dict) -> float:
        x_a = int((A["u1"] + A["u2"])/2)
        y_a = int((A["v1"] + A["v2"])/2)
        x_b = int((B["u1"] + B["u2"])/2)
        y_b = int((B["v1"] + B["v2"])/2)


        w_a = A["u2"] - A["u1"]
        h_a = A["v2"] - A["v1"]
        w_b = B["u2"] - B["u1"]
        h_b = B["v2"] - B["v1"]

        dx = min(A["u2"], B["u2"]) - max(A["u1"], B["u1"])
        dy = min(A["v2"], B["v2"]) - max(A["v1"], B["v1"])


        if dx > 0 and dy > 0:
            return float(dx * dy )/float(w_a * h_a + w_b * h_b - dx*dy)
        else:
            return 0
            

def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = EmptySeatFinder(**vars(opt))
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
        #rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
