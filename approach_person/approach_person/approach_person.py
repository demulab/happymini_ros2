import rclpy
import math
import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node
#from std_msgs.msg import Float32MultiArray #String

import time
import tf_transformations   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 
from happymini_teleop.base_control import BaseControl
from happymini_msgs.srv import TextToSpeech, PersonInArea, LDSDistanceFind

class LDSClient(Node):
    def __init__(self):
        super().__init__('sekkin_client')
        self.pia_srv = self.create_client(LDSDistanceFind, '/fmm/get_lds_distance')
        while not self.pia_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")
        self.pia_srv_req = LDSDistanceFind.Request()

    def lds_send_request(self, text = ''):
        pia_srv_result = 5
        self.pia_srv_req.command = ""      
 
        pia_srv_future = self.pia_srv.call_async(self.pia_srv_req)
        while not pia_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if pia_srv_future.result() is not None:
            pia_srv_result = pia_srv_future.result()
            print(pia_srv_result.distance)
            return pia_srv_result.distance
        else:
            self.get_logger().info(f"Service call failed")
            return None


class SekkinClient(Node):
    def __init__(self):
        super().__init__('sekkin_client')
        self.pia_srv = self.create_client(PersonInArea, 'srv_personinarea')
        while not self.pia_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")
        self.pia_srv_req = PersonInArea.Request()

    def pia_send_request(self, text = ''):
        pia_srv_result = 'None'
        self.pia_srv_req.text = text       
 
        pia_srv_future = self.pia_srv.call_async(self.pia_srv_req)
        while not pia_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if pia_srv_future.result() is not None:
            pia_srv_result = pia_srv_future.result()
            print(pia_srv_result)
            return pia_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None


class Sekkin(Node):
    def __init__(self):
        super().__init__('approach_person')
        #self.sub = self.create_subscription(Float32MultiArray, 'topic', self.Callback, 10)
        #self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.tts_srv = self.create_client(TextToSpeech, 'mimic3_play_server')
        #self.tts_req = TextToSpeech.Request()
        self.srv = self.create_service(TextToSpeech, 'app', self.happymove)
        self.odm_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        #self.master_sub = self.create_subscription(String, 'master', self.mas, 10)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        #self.set_vel(0.0, 0.0)  # 速度の初期化
        self.bc_node = BaseControl()


        self.xx = None
        self.zz = 0
        self.dig = 0
        self.dis = 0 
        self.ang = 0

        self.master = 0

    #def mas(self, srv_req, srv_res):
    #    self.master = srv_req.text

    #def Callback(self, msg):
    #    #self.get_logger().info(f'sub:{msg.data}')
    #    self.xx = msg.data[0]
    #    self.zz = msg.data[2]
    #    #self.dig = - (math.atan2(self.xx, self.zz))*180 / math.pi
    #    #self.get_logger().info(f'{self.dig}')
    #    plt.plot(self.xx, self.zz, '.')
    #    plt.xlim(-1, 1)
    #    plt.ylim(0, 3)
    #    plt.pause(0.1)
    #    #self.happy_move(self.zz, self.dig)
    #    #time.sleep(5)
    #    plt.clf()

    #def tts(self, text):
    #    # Msg
    #    self.tts_req.text = text
        # Call
    #    future = self.tts_srv.call_async(self.tts_req)
    #    while not future.done() and rclpy.ok():
    #        rclpy.spin_once(self, timeout_sec=0.1)
    #    if future.result() is not None:
    #        result = future.result().result
    #        return result
    #    else:
    #        self.get_logger().info("Service call failed")
    #        return False

    def set_sekkin_xy(self):
        self.sekkin_xy = self.sekkinclient.pia_send_request()


    def set_lds_client(self, client):
        self.ldsclient = client

    def set_sekkin_client(self, client):
        self.sekkinclient = client

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

    #def set_vel(self, linear, angular):  # 速度を設定する
    #    self.vel.linear.x = linear   # [m/s]
    #    self.vel.angular.z = angular  # [rad/s]    

    def set_disan(self):
        #self.check()
        self.dis = (self.zz / 2) - 0.3
        print(f"angle : {math.degrees(math.atan2(self.zz, self.xx))}")
        self.ang = -(math.atan2(self.xx, self.zz))*180 / math.pi

    #def timer_callback(self):  # タイマーのコールバック関数
    #    self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 
        
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
            #rclpy.spin_once(self)

   
    def happymove(self, srv_req, srv_res):
        while True:
            kyori = self.sekkinclient.pia_send_request()
            if not kyori.result: 
                #print("waiting for guest arrival")
                #self.tts("waiting for the guest entering.")
                time.sleep(0.5)
            else:
                break


        
        self.xx = kyori.pose.x
        self.zz = kyori.pose.z
        self.master = srv_req.text
        self.get_logger().info(f"{self.master}")
        while rclpy.ok():
            if self.master.find('start') != -1:    
                self.set_disan()
                #self.happy_move(self.dis, self.ang)
                self.bc_node.rotate_angle(self.ang)
                lds_kyori = self.ldsclient.lds_send_request()
                print(f"realsense  distance : {self.dis}m, LDS distance : {lds_kyori}m")


                fused_dis = min(self.dis, max(lds_kyori-0.5, 0.1))
                if self.master.find("nobump") == -1:
                    self.bc_node.translate_dist(fused_dis)
                    time.sleep(1.5)
                    if fused_dis == self.dis:
                        self.bc_node.translate_dist(-0.2, 0.1)
                #self.set_disan()
                #print(self.dis)
                #print(self.ang)
                #self.happy_move(0, self.ang)
                #self.bc_node.rotate_angle(self.ang)
                self.get_logger().info(f'end')
                break
            rclpy.spin_once(self)
        srv_res.result = True
        return srv_res

def main(args = None):
    print('start')
    rclpy.init()
    srv = SekkinClient()
    lds = LDSClient()
    node = Sekkin()
    node.set_sekkin_client(srv)
    node.set_lds_client(lds)
    try:
        while True:
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print('Ctrl+C pushed')
    except ExternalShutdownException:
        sys.exit(1)
    rclpy.shutdown()
