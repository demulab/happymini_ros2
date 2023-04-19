import threading
import math
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class BaseControl(Node):
    def __init__(self):
        super().__init__('base_control_node')
        # Publisher
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Value
        self.twist_value = Twist()
        self.angular_vel = 0.0
        self.target_time = 0.0
        self.target_deg = 0.0
        self.sub_target_deg = 0.0
        self.current_deg = None
        self.judg_deg = 999.9
    
    def odom_callback(self, receive_msg):
        self.angular_vel = receive_msg.twist.twist.angular.z
        quaternion = (
                receive_msg.pose.pose.orientation.x,
                receive_msg.pose.pose.orientation.y,
                receive_msg.pose.pose.orientation.z,
                receive_msg.pose.pose.orientation.w)
        self.current_deg = math.degrees(euler_from_quaternion(quaternion)[2])
        if self.current_deg < 0.0:
            self.current_deg = 180 + (180 - abs(self.current_deg))

    def odom_check(self):
        self.current_deg = None
        while self.current_deg is None and rclpy.ok():
            rclpy.spin_once(self)
            self.get_logger().info("No odom data ...")
            time.sleep(0.7)
        self.get_logger().info("Odom data is available !")

    def round_down(self, value, down_num=1):
        return math.floor(value * (10**down_num)) / (10**down_num)

    def publish_liner_x(self):
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= self.target_time:
            self.twist_pub.publish(self.twist_value)
            end_time = time.time()
            time.sleep(0.1)
        self.twist_value.linear.x = 0.0
        self.twist_pub.publish(self.twist_value)
        self.get_logger().info("Finish 'translate_dist'")
        
    def publish_angular_z(self, max_speed, precision, time_out):
        over_flg = False
        vel_z = 0.0
        vel_max = max_speed
        kp = 0.0073
        ki = 0.0
        kd = 0.0
        start_time = time.time()
        start_plot = time.time()
        while self.round_down(self.judg_deg, precision) != self.round_down(self.current_deg, precision) and rclpy.ok():
            rclpy.spin_once(self)
            plot_time = time.time() - start_plot
            delta_time = time.time() - start_time
            # 0度をまたがないとき
            if self.sub_target_deg == 0.0:
                self.judg_deg = self.target_deg
                if not over_flg and self.target_deg < 20 and self.current_deg > 260:
                    over_flg = True
                    vel_z = 0.3
                    integral_value = 0.0
                elif not over_flg and self.target_deg > 340 and self.current_deg < 100:
                    over_flg = True
                    vel_z = -0.3
                    integral_value = 0.0
                elif over_flg and self.target_deg < 20 and self.current_deg < 20:
                    over_flg = False
                elif over_flg and self.target_deg > 340 and self.current_deg > 340:
                    over_flg = False
                else:
                    pass
                if abs(self.judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.target_deg - self.current_deg)
                else:
                    integral_value = 0
                if not over_flg:
                    vel_z = kp*(self.target_deg - self.current_deg) + ki*integral_value - kd*self.angular_vel
                else:
                    pass
            # 0度を時計回りにまたぐとき
            elif self.sub_target_deg > 180:
                self.judg_deg = self.sub_target_deg
                # 終点が0度通過直後(10度以内)かつフィードバック値が20度未満のとき
                if abs(self.judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.sub_target_deg - self.current_deg)
                else:
                    integral_value = 0
                if self.current_deg > 180:
                    over_flg = True
                if abs(360 - self.sub_target_deg) < 10.0 and 20 - self.current_deg > 0:
                    vel_max = 0.3
                elif self.current_deg < 180 and not over_flg:  # 0度より左側のとき
                    vel_z = -vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*integral_value - kd*self.angular_vel
            # 0度を反時計回りにまたぐとき
            else:
                judg_deg = self.sub_target_deg
                # 終点が0度通過直後のときかつフィードバック値が340度以上のとき
                if abs(judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.sub_target_deg - self.current_deg)
                else:
                    integral_value = 0
                if self.current_deg < 180:
                    over_flg = True
                if abs(0 - self.sub_target_deg) < 10.0 and 360 - self.current_deg < 20:
                    vel_max = 0.3
                elif self.current_deg > 180 and not over_flg:  # 0度より右側のとき
                    vel_z = vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*integral_value - kd*self.angular_vel
            time.sleep(0.05)
            if plot_time > time_out:
                self.get_logger().info("Time out !!!")
                break
            if abs(vel_z) > vel_max:
                vel_z = (vel_z/abs(vel_z))*vel_max
            self.twist_value.angular.z = vel_z
            self.twist_pub.publish(self.twist_value)
        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        self.sub_target_deg = 0.0
        self.get_logger().info(f"Finish deg: {self.round_down(self.current_deg, precision)}")
        self.get_logger().info("Finish 'rotate_angle'")
 
    def translate_dist(self, dist, speed = 0.2):
        try:
            dist = dist.data
        except AttributeError:
            pass
        self.target_time = abs(dist/speed)
        self.twist_value.linear.x = dist/abs(dist)*speed
        self.get_logger().info(f"Start 'translate_dist' >>> {dist}")
        self.publish_liner_x()

    def rotate_angle(self, deg, precision=0, speed=0.7, time_out=10):
        self.odom_check()
        try:
            deg = deg.data
        except AttributeError:
            pass
        time.sleep(0.2)
        if deg >= 0.0:
            self.target_deg = self.current_deg + deg
            self.judg_deg = self.target_deg
            if self.target_deg >= 360:
                self.sub_target_deg = self.target_deg - 360
                self.judg_deg = self.sub_target_deg
            else:
                pass
        else:
            self.target_deg = self.target_deg = self.current_deg + deg
            if self.target_deg < 0.0:
                self.sub_target_deg = 360 + self.target_deg
                self.judg_deg = self.sub_target_deg
            else:
                pass
        self.current_deg = self.round_down(self.current_deg, precision)
        self.target_deg = self.round_down(self.target_deg, precision)
        self.sub_target_deg = self.round_down(self.sub_target_deg, precision)
        self.get_logger().info("Start 'rotate_angle'")
        self.get_logger().info(f"current deg: {self.current_deg}")
        self.get_logger().info(f"target deg: {self.target_deg}")
        self.get_logger().info(f"sub_target deg: {self.sub_target_deg}")
        return self.publish_angular_z(speed, precision, time_out)

    def odom_plot(self, deg, precision=0, speed=0.5, time_out=10):
        time_x = []
        deg_y = []
        plot_time = 0.0
        self.get_logger().info("Start 'odom_plot'")
        rotate_thread = threading.Thread(target=self.rotate_angle, args=(deg, precision, speed, time_out))
        rotate_thread.start()
        time.sleep(0.5)

        start_time = time.time()
        thread_num = len(threading.enumerate())
        while rclpy.ok():
            if thread_num - len(threading.enumerate()) > 0 and time_out < plot_time:
                plt.plot(time_x, deg_y, color='blue', label='Odom data')
                plt.hlines(self.judg_deg, 0, time_out, color='red', linestyles='dotted', label='Target')
                plt.xlabel("Time [sec]")
                plt.ylabel("Degree [deg]")
                plt.xlim(0, time_out)
                plt.ylim(min(deg_y) - 30, self.judg_deg + 30)
                plt.legend()
                break
            plot_time = time.time() - start_time
            # グラフデータ作成
            time_x.append(plot_time)
            deg_y.append(self.current_deg)
            # 軸ラベル
            plt.xlabel("Time [sec]")
            plt.ylabel("Degree [deg]")
            # プロット範囲
            plt.xlim(0, time_out)
            plt.ylim(min(deg_y) - 30, self.judg_deg + 30)
            # プロット
            plt.plot(time_x, deg_y, color='blue', label='Odom data')
            plt.hlines(self.judg_deg, 0, time_out, color='red', linestyles='dotted', label='Target')
            # 凡例表示
            plt.legend()
            # 0.1秒間隔で更新
            plt.pause(0.1)
            # 画面初期化
            plt.clf()
        self.get_logger().info("All plotted!")
        plt.show()


def main():
    rclpy.init()
    bc = BaseControl()
    try:
        #thread = threading.Thread(target=rclpy.spin, args=(bc, ), daemon=True)
        #thread.start()
        time.sleep(0.1)
        #bc.translate_dist(0.3)
        #bc.rotate_angle(30)
        bc.odom_plot(-90, 1, 0.5, 10)
    except KeyboardInterrupt:
        pass
    #thread.join()
    rclpy.shutdown()
