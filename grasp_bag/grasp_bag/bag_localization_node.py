import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from grasp_bag.scan_data_sensing_node import ScanDataSensing
from happymini_msgs.srv import BagLocalization
import os
import sys
import math
import time
import threading


class BagLocalizationServer(Node):
    def __init__(self):
        super().__init__('bag_localization_node')
        # Service
        self.srv = self.create_service(BagLocalization, 'bag_localization_server', self.estimation_execute)
        # Value
        self.search_range_data = []
        self.scan_custom_range = 999.9
        # Module
        self.sds_node = ScanDataSensing()
        self.get_logger().info("Ready to set bag_localization_server")

    def average(self, input_list):
        return sum(input_list)/len(input_list)

    def command_to_range(self, left_right):
        if left_right == 'right':
            self.search_range_data = self.sds_node.scan_custom_data[: self.sds_node.scan_custom_center + 1]
        elif left_right == 'left':
            self.search_range_data = self.sds_node.scan_custom_data[self.sds_node.scan_custom_center :]
        elif left_right == 'all':
            self.search_range_data = self.sds_node.scan_custom_data
        else:
            self.get_logger().error("Please input 'left' or 'right' or 'all'")
            rclpy.shutdown()
            quit()

    def bag_range_estimation(self):
        dispersion_param = 0.01
        laser_deviation = 0
        result_dict = {}
        data_list = []
        bag_range = []
        # データ分割
        self.get_logger().info("Estimating position from scan data ...")
        min_index = self.search_range_data.index(min(self.search_range_data))
        index_num = min_index
        for data in self.search_range_data[min_index :]:
            data_list.append(data)
            laser_average = self.average(data_list)
            laser_deviation += (data - laser_average)**2
            laser_dispersion = laser_deviation/len(data_list)
            if laser_dispersion < dispersion_param:
                bag_range.append(index_num)
            elif laser_average != 'NULL' and laser_dispersion >= dispersion_param:
                laser_deviation = 0
                data_list.clear()
                break
            else:
                pass
            index_num += 1
            time.sleep(0.05)
        self.search_range_data.reverse()
        index_num = min_index - 1
        for data in self.search_range_data[self.search_range_data.index(min(self.search_range_data)) + 1 :]:
            data_list.append(data)
            laser_average = self.average(data_list)
            laser_deviation += (data - laser_average)**2
            laser_dispersion = laser_deviation/len(data_list)
            if laser_dispersion < dispersion_param:
                bag_range.insert(0, index_num)
            elif laser_average != 'NULL' and laser_dispersion >= dispersion_param:
                break
            index_num -= 1
            time.sleep(0.05)
        self.search_range_data.reverse()
        result_dict['bag_range'] = bag_range
        result_dict['bag_center'] = bag_range[self.sds_node.round_half_up(len(bag_range)/2 - 1)]
        return result_dict
    
    def range_to_angle(self, left_right, bag_dict):
        if left_right == 'left':
            angle_to_bag = (len(self.search_range_data) - 1) - bag_dict['bag_center']
        elif left_right == 'right':
            angle_to_bag = bag_dict['bag_center']
        else:
            angle_to_bag = bag_dict['bag_center'] - self.sds_node.scan_custom_center
        distance_to_bag = self.sds_node.scan_custom_data[bag_dict['bag_center']]
        self.get_logger().info(f"Angle to bag >>> {angle_to_bag}")
        self.get_logger().info(f"Distance to bag >>> {distance_to_bag}")
        return float(angle_to_bag), float(distance_to_bag)

    def estimation_execute(self, srv_req, srv_res):
        self.sds_node.scan_range_set(srv_req.degree)
        self.command_to_range(srv_req.left_right)
        bag_dict = self.bag_range_estimation()
        angle_to_bag, distance_to_bag = self.range_to_angle(srv_req.left_right, bag_dict)
        if srv_req.graph:
            self.sds_node.graph_plot(srv_req.degree, self.search_range_data, bag_dict)
        srv_res.angle_to_bag = angle_to_bag
        srv_res.distance_to_bag = distance_to_bag
        return srv_res

def main():
    rclpy.init()
    bls_node = BagLocalizationServer() 
    try:
        rclpy.spin(bls_node)
    except KeyboardInterrupt:
        pass
    bls_node.destroy_node()
    bls_node.sds_node.destroy_node()
    rclpy.shutdown()
