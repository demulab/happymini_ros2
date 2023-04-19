import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math
import time
import threading
import matplotlib.pyplot as plt


class ScanDataSensing(Node):
    def __init__(self):
        super().__init__('scan_data_sensing_node')
        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # Value
        self.scan_data = []
        self.scan_index_list = []
        self.scan_increment = 999.9
        self.scan_custom_data = []
        self.scan_custom_center = 'NULL'

    def scan_callback(self, receive_msg):
        self.scan_data = list(receive_msg.ranges)
        self.scan_increment = math.degrees(receive_msg.angle_increment)

    def deg_to_index(self, deg):
        return int(deg / self.scan_increment)

    def round_half_up(self, value):
        decimals = math.modf(value)[0]
        return int(value + 1) if decimals >= 0.5 else int(value)

    def scan_check(self):
        self.scan_data.clear()
        while not self.scan_data and rclpy.ok():
            rclpy.spin_once(self)
            self.get_logger().info("No scan data ...")
            time.sleep(0.7)
        self.get_logger().info("Scan data is available !")

    def scan_params(self):
        scan_index_sum = len(self.scan_custom_data)
        self.scan_custom_center = self.round_half_up(scan_index_sum/2)
        self.get_logger().info(f"Number of scan data >>> {scan_index_sum}")
        self.get_logger().info(f"Center of scan index >>> {self.scan_custom_center}")
        self.get_logger().info(f"Degree per step >>> {self.scan_increment}")

    def scan_range_set(self, deg):
        self.scan_check()
        local_scan_data = self.scan_data
        target_range = self.deg_to_index(deg)
        max_index = int(target_range / 2)  # 残すデータの終了位置
        min_index = 360 - max_index  # 残すデータの開始位置(時計回り)
        del local_scan_data[max_index + 1 : min_index]  # 必要なデータ以外削除
        # 中心からスキャンしてる場合は入れ替え
        tmp_list = local_scan_data[0 : max_index + 1]
        del local_scan_data[0 : max_index + 1]
        local_scan_data.extend(tmp_list)
        # インスタンス変数に格納
        self.scan_custom_data = self.scan_zero_change(local_scan_data)
        self.scan_params()

    def scan_zero_change(self, in_zero_list):
        changed_list = []
        for value in in_zero_list:
            if value >= 0.2:
                one_buck_value = value
                changed_list.append(value)
            #elif value == 0.0:
            #    changed_list.append(3.0)
            else:
                changed_list.append(one_buck_value)
        return changed_list

    def graph_data_generate(self, scan_data):
        for i in range(len(scan_data)):
            self.scan_index_list.append(i)

    def graph_plot(self, deg=360, scan_data=None, estimate_result=None):
        if scan_data is None:
            self.scan_range_set(deg)
            scan_data = self.scan_custom_data
        else:
            pass
        self.graph_data_generate(scan_data)
        self.get_logger().info("Plotting scan data")
        plt.plot(self.scan_index_list, scan_data)
        if estimate_result is not None:
            plt.vlines(estimate_result['bag_range'][0], 0, 4, color='red', linestyles='dotted')
            plt.vlines(estimate_result['bag_range'][-1], 0, 4, color='red', linestyles='dotted')
            plt.vlines(estimate_result['bag_center'], 0, 4, color='green', linestyles='dotted')
        else:
            pass
        plt.show()
        #if plt.waitforbuttonpress():
        #    plt.close()


def main():
    rclpy.init()
    sds_node = ScanDataSensing()
    try:
        time.sleep(0.1)
        sds_node.graph_plot(180)
    except KeyboardInterrupt:
        pass
    #thread.join()
    sds_node.destroy_node()
    rclpy.shutdown()
