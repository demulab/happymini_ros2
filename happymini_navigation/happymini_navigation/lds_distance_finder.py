import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2 
from happymini_msgs.srv import LDSDistanceFind
import math
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data


class LDSDistanceFinder(Node):
    def __init__(self):
        super().__init__("lds_distance_finder")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)
        
        self.srv_personinmap = self.create_service(LDSDistanceFind, '/fmm/get_lds_distance', self.get_lds_distance)


    def scan_callback(self, msg):
        self.scan = msg


    def get_lds_distance(self, req, res):
        distance_resolution = 0.01
        robot_radius = 0.4
        lds_img = np.zeros((500, 500))
        lds_img.fill(255)

        center = (0, 250)
        h, w = lds_img.shape

        for i in range(len(self.scan.ranges)):
            angle = self.scan.angle_min + self.scan.angle_increment * i + math.pi/2
            diff_x = self.scan.ranges[i] * math.cos(angle)
            diff_y = self.scan.ranges[i] * math.sin(angle)
            #print("angle :", math.degrees(angle))
            #print("x , y =",diff_x, diff_y)
            x_px = int(diff_x / distance_resolution)
            y_px = int(diff_y/ distance_resolution)
            if center[0] + y_px < 0:
                continue
            elif center[0] + y_px > h -1:
                continue
            elif center[1] + x_px < 0:
                continue
            elif center[1] + x_px > w -1:
                continue
            lds_img[center[0]+y_px, center[1]+x_px] = 0

        cv2.imwrite("/home/demulab/test_data/lidar_scan.png", lds_img)

        robot_radius_px = int(robot_radius/distance_resolution)

        cropped_lds = lds_img[:, center[1]-robot_radius_px:center[1]+robot_radius_px]
        cv2.imwrite("/home/demulab/test_data/cropped.png", cropped_lds) 
        find_min = False 
        height_idx = 500-1



        for i in range(len(cropped_lds)):
            for j in range(len(cropped_lds[i])):
                print(cropped_lds[i, j])
                if cropped_lds[i, j] == 0:
                    print("fiind minima")
                    find_min = True
                    height_idx = i
                    break
            if find_min:
                break
                

        lds_distance = height_idx * distance_resolution

        res.distance =lds_distance
        return res




def main():
    rclpy.init()
    node = LDSDistanceFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
