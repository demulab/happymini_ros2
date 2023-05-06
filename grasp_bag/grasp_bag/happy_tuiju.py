import rclpy
import math
import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

import time
import tf_transformations   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 
from happymini_teleop.base_control import BaseControl


class Tuiju(Node):
    def __init__(self):
        super().__init__('tuiju_suv')
        self.sub = self.create_subscription(Float32MultiArray, 'topic', self.Callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odm_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.master_sub = self.create_subscription(String, 'master', self.mas, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.set_vel(0.0, 0.0)  # 速度の初期化
        self.bc_node = BaseControl()


        self.xx = None
        self.zz = 0
        self.dig = 0
        self.dis = 0 
        self.ang = 0

        self.master = 0

    def mas(self, msg):
        self.master = msg.data

    def Callback(self, msg):
        #self.get_logger().info(f'sub:{msg.data}')
        self.xx = msg.data[0]
        self.zz = msg.data[2]
        #self.dig = - (math.atan2(self.xx, self.zz))*180 / math.pi
        #self.get_logger().info(f'{self.dig}')
        plt.plot(self.xx, self.zz, '.')
        plt.xlim(-1, 1)
        plt.ylim(0, 3)
        plt.pause(0.1)
        #self.happy_move(self.zz, self.dig)
        #time.sleep(5)
        plt.clf()

    def check(self):
        self.xx = None
        while self.xx is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info("No yoko")
        self.get_logger().info("available")


    def get_pose(self, msg):      # 姿勢を取得する                                               
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw

    def odom_cb(self, msg):         # オドメトリのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        #self.get_logger().info(
        #    f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad]')

    def set_vel(self, linear, angular):  # 速度を設定する
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    

    def set_disan(self):
        self.check()
        self.dis = (self.zz / 2) - 0.3
        self.ang = - (math.atan2(self.xx, self.zz))*180 / math.pi

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 
        
    def happy_move(self, distance, angle):# 簡単な状態遷移
        state = 0
        self.get_logger().info(f'angle{angle}')
        
        self.get_logger().info(f'distance{distance}')
        
        #time.sleep(0.5)
        #rclpy.spin_once(self)
        while rclpy.ok():
            if state == 0:
                self.bc_node.rotate_angle(angle)
                state = 1
            elif state == 1:
                self.bc_node.translate_dist(distance, speed = 0.1)
                state = 2
            elif state == 2:
                break  
            else:
                print('エラー状態')
            rclpy.spin_once(self)

    def happymove(self):
        rclpy.spin_once(self)
        while rclpy.ok():
            if self.master == 'start':    
                self.set_disan()
                self.happy_move(self.dis, self.ang)
                #self.bc_node.translate_dist(-0.2, 0.1)
                #self.set_disan()
                #self.happy_move(0, self.ang)
                self.get_logger().info(f'end')
            rclpy.spin_once(self)

def main(args = None):
    print('start')
    rclpy.init()
    node = Tuiju()
    try:
        print('waite')
        node.happymove()
        print('owari')
        #node.happy_move()
    except KeyboardInterrupt:
        print('Ctrl+C pushed')
    except ExternalShutdownException:
        sys.exit(1)
    rclpy.shutdown()
