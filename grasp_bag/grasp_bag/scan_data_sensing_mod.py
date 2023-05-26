import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math
import time
import threading
import matplotlib.pyplot as plt


class ScanDataSensing(Node):
    def __init__(self, lrf_type, up_down):
        super().__init__('scan_data_sensing_node')
        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # Value
        self.scan_data = []
        self.scan_index_list = []
        self.scan_increment = 999.9
        self.scan_custom_data = []
        self.scan_custom_center = 'NULL'
        self.lrf_info = {'LRF_TYPE': lrf_type, 'UP_DOWN': up_down}
        self.get_logger().info(f"LRF_TYPE: {self.lrf_info['LRF_TYPE']}")
        self.get_logger().info(f"UP_DOWN: {self.lrf_info['UP_DOWN']}")

    def scan_callback(self, receive_msg):
        self.scan_data = list(receive_msg.ranges)
        self.scan_increment = math.degrees(receive_msg.angle_increment)
        if self.lrf_info['UP_DOWN'] == 'down' or self.lrf_info['UP_DOWN'] == 'DOWN':
            self.scan_data = list(reversed(self.scan_data))

    def deg_to_index(self, deg):
        return int(deg / self.scan_increment)

    def round_half_up(self, value):
        decimals = math.modf(value)[0]
        return int(value + 1) if decimals >= 0.5 else int(value)

    def scan_check(self):
        self.scan_data.clear()
        while not self.scan_data and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.7)
            self.get_logger().info("No scan data ...")
        self.get_logger().info("Scan data is available !")

    def scan_params(self):
        scan_index_sum = len(self.scan_custom_data)
        self.scan_custom_center = self.round_half_up(scan_index_sum/2)
        self.get_logger().info(f"Number of scan data: {scan_index_sum}")
        self.get_logger().info(f"Center of scan index: {self.scan_custom_center}")
        self.get_logger().info(f"Degree per step: {self.scan_increment}")

    def scan_range_set(self, deg):
        self.scan_check()
        local_scan_data = self.scan_data
        target_range = self.deg_to_index(deg)
        # LRFによってリスト調整
        if self.lrf_info['LRF_TYPE'] == 'LDS-01':  # 中心からスキャンしてるので入れ替え
            max_index = int(target_range / 2)  # 残すデータの終了位置
            min_index = 360 - max_index  # 残すデータの開始位置(時計回り)
            del local_scan_data[max_index + 1 : min_index]  # 必要なデータ以外削除
            tmp_list = local_scan_data[0 : max_index + 1]
            del local_scan_data[0 : max_index + 1]
            local_scan_data.extend(tmp_list)
            self.scan_custom_data = self.scan_zero_change(local_scan_data)
        elif self.lrf_info['LRF_TYPE'] == 'UTM-30LX':
            delete_range = int((len(local_scan_data) - target_range) / 2)
            if delete_range == 0:
                pass
            else:
                del local_scan_data[0 : delete_range + 1]
                del local_scan_data[target_range + 1:]
            self.scan_custom_data = local_scan_data
        else:
            self.get_logger().info(f"'{self.lrf_info['LRF_TYPE']}' not supported")
        self.scan_params()

    def scan_zero_change(self, in_zero_list):
        changed_list = []
        one_back_value = 3.0
        for value in in_zero_list:
            if value >= 0.2:
                one_back_value = value
                changed_list.append(value)
            else:
                changed_list.append(one_back_value)
            time.sleep(0.001)
        return changed_list

    def graph_data_generate(self, scan_data):
        for i in range(len(scan_data)):
            self.scan_index_list.append(i)
            time.sleep(0.001)

    def graph_plot(self, deg=180, scan_data=None, estimate_result=None):
        if scan_data is None:
            self.scan_range_set(deg)
            scan_data = self.scan_custom_data
        else:
            pass
        self.graph_data_generate(scan_data)
        self.get_logger().info("Plotting scan data")
        # 軸ラベル
        plt.xlabel("Number of data")
        plt.ylabel("Distance [m]")
        # プロット
        plt.plot(self.scan_index_list, scan_data, color='blue', label='Scan data')
        if estimate_result is not None:
            plt.vlines(estimate_result['bag_range'][0], 0, 4, color='red', linestyles='dotted', label='Edge of object')
            plt.vlines(estimate_result['bag_range'][-1], 0, 4, color='red', linestyles='dotted')
            plt.vlines(estimate_result['bag_center'], 0, 4, color='green', linestyles='dotted', label='Center of object')
        else:
            pass
        plt.legend()  # 凡例表示
        plt.show()
        self.scan_index_list.clear()


def main():
    rclpy.init()
    sds_node = ScanDataSensing()
    try:
        sds_node.graph_plot(180)
    except KeyboardInterrupt:
        pass
    sds_node.destroy_node()
    rclpy.shutdown()
