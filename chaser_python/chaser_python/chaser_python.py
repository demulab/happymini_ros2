#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import cv2 
import sys
import numpy as np
from enum import Enum
import math
from matplotlib import pyplot as plt
import copy

def DEG2RAD(DEG): return math.pi*DEG/180.0
def RAD2DEG(RAD): return 180.0*RAD/math.pi

g_find_leg_radius = 20.0                #検出範囲の中心座標から半径[px]
kExtendRadius = 30.0                    #見失ったときの検出範囲[px]
kOrignalRadius = g_find_leg_radius      
kMagnificationWorldImagePos = 10.0      #画像上で物体同士が重ならないようにするための世界座標の倍率
kUpdateLastImageCount  = 30.0           #比較する世界座標系の画像を更新するループ回数　one loop 30[ms]

kFollowMaxDistance = 2.0                #追従距離の最大
kFollowDistance    = 0.4                #人からこの距離で追従する
kFollowMinDistance = 0.3                #追従距離の最小
kFollowAngle       = 180.0              #探す範囲は正面のこの角度[deg]
kGainLinear        = 0.7                #P制御比例ゲイン
kKp                = 0.8                #PD制御ゲイン(回転)
kKd                = 0.5                #
kLinearMaxSpeed    = 0.3                #並進の最大速度
kTurnMaxSpeed      = 0.7                #角速度　最大3.14[rad/s]

kDefaultDetectPosX	= 250.0             #検出範囲のxの初期の中心座標[px]
kDefaultDetectPosY = 200.0              #検出範囲のyの初期の中心座標[px]
kLostTime          = 60.0               #人を完全に見失ったと判断するループ回数、one loop 30[ms]
kLegBetweenDistance = 0.4               #人の足だと判断する足候補感の距離[m]

kImageWidth=500                         #[px]
kImageHeight=500                        #[px]

kMToPixel = 50.0                        #mをpixilへ変換 1[m] = 50 pixel 1[pixel] = 2[cm]

#最大値と最小値の指定関数
def floatlimit(min,value,max):
    if value > max: return max
    elif value < min: return min
    else: return value

#白
firstColor = 255

#初期化時に塗りつぶす
lidar_image = np.zeros((kImageWidth,kImageHeight,3),np.uint8)
lidar_gray_image = np.zeros((kImageWidth,kImageHeight),np.uint8)+firstColor

#脚候補の世界系座標
world_pose_candidata_leg_image = np.zeros((500,500),np.uint8)+firstColor

#ライダーからのデータを世界座標系にした画像
world_pose_image = np.zeros((500,500),np.uint8)+firstColor
last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
erode_last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
opening_last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#世界座標系の画像の差分画像
substraction_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#検出範囲外の脚候補の位置を世界座標系にして描写する画像
static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
erode_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
opening_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#LiDAR画像に縮小処理
lidar_erode_image = np.zeros((500,500),np.uint8)

#色
red = (0,0,255)
blue = (255,0,0)
green = (0,255,0)

SARE = 0
DANGER = 119

#「人についてのクラス」
#人までの距離や角度の情報をメンバとして宣言

class Pose():
    x = 0.0
    y = 0.0
    z = 0.0
    theta = 0.0

class Object():
    #初期化コンストラクタ
    def __init__(self):
        #c++ではprivate
        self._x_ = 0.0
        self._y_ = 0.0
        self._z_ = 0.0
        self._theta_ = 0.0

        #ローカル座標、ワールド座標
        self.local = Pose()
        self.world = Pose()

        #反射強度の最小、最大値[0:255]
        self.intensity_min = 0
        self.intensity_max = 255

        #距離
        self.distance = 0.0
        self.last_distance = 0.0
        #角度[rad/s]
        self.angle = 0.0
        self.last_angle = 0.0

        #半径
        self.radius = 0.0
        #画像座標系での位置
        self.image_pos = Pose()

        #yamada
        #１時刻前の人の位置
        self.last_human_x = kDefaultDetectPosX
        self.last_human_y = kDefaultDetectPosY
        #予測される人の位置
        self.image_expect_human_x = self.image_expect_human_y = 0.0
        #動体と予測した物体の位置
        self.dynamic_image_pos = Pose()
        #１時刻前の速度
        self.last_linear_speed = 0.0

        #動体かの判断
        self.judge_dynamic = False

    def getX(self): return self._x_
    def getY(self): return self._y_
    def getZ(self): return self._z_
    def getTheta(self): return self._theta_
    def setX(self,x): self._x_ = x
    def setY(self,y): self._y_ = y
    def setZ(self,z): self._z_ = z
    def setTheta(self,theta): self._theta_ = theta

human = Object()

#「ロボットやセンサについてのクラス」
# LiDARの情報やロボットの情報をメンバとして宣言

class Robot(Node):
    #初期化コンストラクタ
    def __init__(self):
        super().__init__('chaser_python')
        #人を見失った回数
        self.lost_count = 0

        #人を見失っているか
        self.human_lost = True
        #位置[m],向き[rad]
        self.robot_x = self.robot_y = self.robot_theta = 0.0
        #hokuyo LiDAR UTM-30LX
        self.laser_distance = np.zeros(1081)
        self.laser_last_distance = np.zeros(1081)
        self.laser_intensities = np.zeros(1081)
        self.laser_last_intensities = np.zeros(1081)

        #1時刻前のx,y,theta
        self.last_x = 0.0
        self.last_y = 0.0 
        self.last_theta = 0.0
        #[m/s],[rad/s]
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.cmd_vel = self.zero_cmd_vel = Twist()
        self.find_fuman_msg = Bool()
        self.chaser_check_msg = Bool()

        #画像のサイズを超えて書き込もうとした際、その座標を中心からの座標に更新するための値
        self.displace_x = self.displace_y = 0.0
        #このカウントがkUpdateLastImageCountと同じになればlast_world_pose_imageを更新、ver2
        self.last_image_count = 0

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.chaser_check = self.create_publisher(Bool, 'chaser_check', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laserCallback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odomCallback, 10)
        self.find_human_pub = self.create_publisher(Bool, 'find_human', 10)
        self.follow_human_sub = self.create_subscription(String, 'follow_human', self.followHumanCallback, 10)
        #public
        #ローカル座標
        self.local = Pose()
        self.world = Pose()
        self.dataCount = 0
        self.laser_angle_min = self.laser_angle_max = 0
        self.follow_command = "false"

    '''
    laser_callbackの重い部分
    '''

    def laser_cycle(self):
        #callback関数に一度でも入っていたら処理をする
        if not (self.dataCount == 0):
            #LiDARのデータを画像に変換
            self.changeToPicture(self.dataCount,self.laser_angle_min,self.laser_angle_max)
            
            self.laser_last_distance = self.laser_distance.copy
            self.laser_last_intensities = self.laser_intensities.copy

            self.prepWindow()

    def getLinearSpeed(self): return self.linear_speed
    def getAngularSpeed(self): return self.angular_speed

    def welcomeMessage(self):
        self.get_logger().info("Follow me program by demura lab.,KIT")
        self.get_logger().info(f"{kFollowMaxDistance}")
        self.get_logger().info(f"{kFollowMinDistance}")
        self.get_logger().info(f"{kFollowAngle}")

    '''
    追従速度の設定をする関数
    人までの距離がほしいのでObject humanを引数にする
    return:並進速度
    人からkFollowDistance[m]後ろに離れた位置にP制御で追従
    '''
    def followLinearSpeed(self,human_obj):
        speed = ((human_obj.distance - kFollowDistance) * kGainLinear)

        #速度の上限と加減を設定
        if speed > kLinearMaxSpeed:
            speed = kLinearMaxSpeed

        #近づきすぎたら停止
        if speed < 0.1:
            speed = 0.0

        return speed

    '''
    脚の探索範囲の中心位置を初期位置に戻す関数
    human_obj   ： 1時刻前の人の一を使うので引数にする
    時刻前の脚の位置を初期化する
    人を見失った状態なのでlost_linear_speedを0にする
    '''

    def defaultPos(self,human_obj):
        #グローバル宣言
        global static_object_world_pose_image,opening_static_object_world_pose_image
        #1時刻前の座標
        human_obj.last_human_x = kDefaultDetectPosX
        human_obj.last_human_y = kDefaultDetectPosY
        #1時刻前の速度
        human_obj.last_linear_speed = 0.0

        static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
        opening_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

    '''
    画像に世界座標系にしたライダーからのデータを書き込む関数
    point_x point_y 書き込む座標
    画像より大きい座標を書き込む場合、初期位置(画像の中心)にまでずらして書き込む
    ROS似合わせているので、xが縦軸、yが横軸
    '''
    def writeWorldPoseImages(self,point_x,point_y):
        #グローバル関数使用宣言
        global static_object_world_pose_image,world_pose_image

        if point_x < 0:
            #静止物体の位置を示す画像の初期化
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            #初期位置(画像の中心)からの座標にずらす
            self.displace_x += kImageHeight/2.0
            point_x += self.displace_x
            #世界座標系にしたLiDARデータを書き込む
            world_pose_image[int(point_x)][int(point_y)] = 0 
        elif point_x > kImageHeight:
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_x -= kImageHeight/2.0
            point_x -= self.displace_x
            world_pose_image[int(point_x)][int(point_y)] = 0
        elif (point_x >= 0) and (point_x <= kImageHeight) and (point_y < 0):
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_y += kImageWidth/2.0
            point_y += self.displace_y
            world_pose_image[int(point_x)][int(point_y)] = 0
        elif (point_x >= 0) and (point_x <= kImageHeight) and (point_y > kImageWidth):
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_y -= kImageWidth/2.0
            point_y -= self.displace_y
            world_pose_image[int(point_x)][int(point_y)] = 0
        else:
            #世界座標系に変換したLiDARからのデータを書き込む
            world_pose_image[int(point_x)][int(point_y)] = 0

    '''
    減速する関数
    減速するために１時刻前の速度を使うので、Object humanを引数にする
    １時刻前の速度を使って減速する。速度が0.1[m/s]以下になったら停止
    '''

    def reduceSpeed(self,human_obj):
        #徐々に速度を落とす
        human_obj.last_linear_speed = human_obj.last_linear_speed*0.8
        self.setLinearSpeed(human_obj.last_linear_speed)

        if human_obj.last_linear_speed <= 0.1:
            self.setLinearSpeed(0.0)
            self.setAngularSpeed(0.0)
            human_obj.distance = 999.0
            human_obj.angle = 999.0


    '''
    並進速度をLinear_speedに設定する関数
    linear:平真速度[m/s]
    取得するときはgetLinearSpeed()を使用する
    '''
    def setLinearSpeed(self,linear): 
        self.linear_speed = linear
    
    '''
    角速度をangular_speedに設定する関数
    angular:角速度[m/s]
    取得するときはgetAngularSpeed()を使用する
    '''
    def setAngularSpeed(self,angular): 
        self.angular_speed = angular

    '''
    並進速度[m/s]と角速度[rad/s]をパブリッシュする
    linear:平真速度[m/s]
    angular:角速度[m/s]
    '''
    def move(self,linear,angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        print(cmd)
        time.sleep(0.01)
        self.cmd_vel_pub.publish(cmd)

    '''
    opencvのAPIを使うためにLiDARデータを画像に変換する関数
    dataCount：走査線数
    laser_angle_min,laser_angle_max LiDARの走査線の一番右(最小)と一番左(最大)
    lidar_image(500*500[px])の範囲に反応があったら白(255),なければ黒(0)をlidar_gray_imageの各要素に入力する
    '''

    def changeToPicture(self,dataCount,laser_angle_min,laser_angle_max):
        #グローバル関数使用宣言
        global lidar_gray_image,world_pose_candidata_leg_image,world_pose_image,last_world_pose_image
        global opening_last_world_pose_image

        #画像を黒一色で初期化
        lidar_gray_image = np.zeros((500,500),np.uint8)

        #中央の走査線番号
        center = dataCount/2
        #kFollowAngle(前方180度)の走査線数
        search_lines = int(1080*(kFollowAngle/270.0))

        local = Pose()
        world = Pose()

        for j in range(int(center - search_lines/2),int(center + search_lines/2)+1):
            #捜査線の角度
            angle = (laser_angle_max-laser_angle_min)*j/float(dataCount)+laser_angle_min
            #画像座標系に変換
            tmp = int(kMToPixel * self.laser_distance[j] * math.cos(angle))
            #rosは進行方向がx,横がy
            x = int(kImageWidth/2 - kMToPixel * self.laser_distance[j] * math.sin(angle))   #x軸は左右反転
            y = int(kImageHeight/2 - tmp)                                                   #y軸は上下反転

            #VER2
            #ローカル座標系はROSに合わせて進行方向がx,左方向がy
            local.y = float(x - kImageWidth/2)/float(kMToPixel)
            local.x = float(y - kImageHeight/2)/float(kMToPixel)
            #ワールド座標系に変換
            self.localToWorld(local,world)

            #cols:横画素数500,rows：縦画素数500
            if (0 <= x) and (x < lidar_image.shape[1]) and (0 <= y) and (y < lidar_image.shape[0]):
                #6000は反射強度の最大値
                value = int(self.laser_intensities[j]*255.0/6000.0)
                if value > 255: 
                    value = 255
                if value < 0: 
                    value = 0

                #グレースケール画像に正規化した反射強度valueを代入
                lidar_gray_image[int(y),int(x)] = value

                #VER2
                #世界座標系
                point_x = kMagnificationWorldImagePos * world.x + kImageWidth/2 + self.displace_x
                point_y = kMagnificationWorldImagePos * world.y + kImageHeight/2 + self.displace_y
                self.writeWorldPoseImages(point_x,point_y)

        #VER2
        #動体を検出するための処理
        #差分をとるワールド座標系の画像をlast_image_countごとに更新
        if self.last_image_count >= kUpdateLastImageCount:
            last_world_pose_image = copy.deepcopy(world_pose_image)
            #画像を収縮して膨張、ノイズを除去
            opening_last_world_pose_image=cv2.morphologyEx(last_world_pose_image,cv2.MORPH_OPEN,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)
            self.last_image_count = 0
            #画像を初期化
            world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            world_pose_candidata_leg_image = np.zeros((500,500),np.uint8)+firstColor

        self.last_image_count+=1

    '''
    LiDARデータを受け取るコールバック関数
    data:トピック/scanをサブスクライブしている
    LiDARの捜査線数や距離、反射強度の値を変数に代入。LiDARデータをchangeToPictureに渡す
    '''

    def laserCallback(self,data):
        self.a = 0
        #捜査線数
        self.dataCount = np.size(data.ranges)
        #LiDARの一番右の走査線-2.356194[rad] -135度
        self.laser_angle_min = data.angle_min
        #LiDARの一番右の走査線 2.356194[rad]  135度
        self.laser_angle_max = data.angle_max

        self.laser_angle_max

        #最大60[m] 最小0.023[m]
        self.laser_distance = np.array([999 if (value <= data.range_min or value >= data.range_max) else value for value in data.ranges])
        #反射強度
        self.laser_intensities = np.array([-999 if self.laser_distance[i] == 999 else data.intensities[i] for i in range(self.dataCount)])

    '''
    ローカル座標系をワールド座標系に変換
    local_pose:ローカル座標
    world_pose:ワールド座標
    ワールド座標系での物体の位置＝回転行列*ローカル座標の物体の位置 - ローカル座標の原点(ワールド座標でのロボットの位置)
    '''

    def localToWorld(self,local_pose,world_pose):
        world_pose.x = local_pose.x * math.cos(self.robot_theta) - local_pose.y * math.sin(self.robot_theta) - self.robot_x
        world_pose.y = local_pose.x * math.sin(self.robot_theta) + local_pose.y * math.cos(self.robot_theta) - self.robot_y
        world_pose.theta = self.robot_theta

    '''
    入力画像から脚候補の数を探索する関数
    input_image     : 入力画像
    objects         : 脚候補のオブジェクト
    display_image   : 出力画像
    color           : 色
    contour_min     : 輪郭長            ：最小[px]
    contour_max     : 輪郭長            ：最大[px]
    width_min       : 外接矩形          ：最小[px]
    width_max       : 外接矩形          ：最大[px]
    ratio_min       : 矩形横縦          ：最小[px]
    ratio_max       : 矩形横縦          ：最大[px]
    m00_min         : 0次モーメント     ：最小[px]
    m00_max         : 0次モーメント     ：最大[px]
    m01_min         : 1次モーメント     ：最小[px]
    m01_max         : 1次モーメント     ：最大[px]
    diff_x          : 外接円と重心の差
    diff_y          : 外接円と重心の差
    各パラメータを調節して脚候補となるオブジェクトを調節できる
    脚候補は輪郭で判断する、条件に合致した輪郭を脚候補として数え合計の値を返す
    動体を検出する処理を追加、予めわかっている脚に似た物体を脚候補から除外
    '''

    def findLegs(self,input_image,objects,display_image,color,contour_min, contour_max,width_min,width_max,ratio_min,ratio_max,m00_min,m00_max,m10_min,m10_max,diff_x,diff_y):
        #グローバルの使用宣言
        global lidar_gray_image,substraction_world_pose_image,world_pose_candidata_leg_image,opening_last_world_pose_image

        #輪郭の探索
        contours, hierarchy = cv2.findContours(input_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE,offset = (0,0))
        
        #脚候補の数
        object_num = 0

        for cn in range(0,len(contours)):
            tmp = 0
            count = 0
            intensity = 0
            #最小外接円の中心
            center = Pose()
            #輪郭の最小外接円を取得
            (center.x,center.y),radius = cv2.minEnclosingCircle(contours[cn])

            #ロボットより後ろは除外
            if center.y - kImageHeight/2.0 > 0:
                continue

            #輪郭の長さにより除外
            #print(len(contours[cn]))  # contour_min, contour_maxをこれみて調整
            if not ((len(contours[cn]) >= contour_min) and (len(contours[cn]) <= contour_max)):
                continue
            #print(len(contours[cn]))  # contour_min, contour_maxをこれみて調整

            #kFollowMaxDistance * kMToPixelより遠い物体は検出しない
            if kFollowMaxDistance * float(kMToPixel) < kImageHeight/2.0 - center.y:
                continue
            
            #概説する長方形を求める
            rect_x,rect_y,rect_w,rect_h = cv2.boundingRect(contours[cn])

            #長方形の底面による除外
            #print(f"rect_w: {rect_w}")  # width_min, width_maxはこれみて調整
            if not ((rect_w >= width_min) and (rect_w <= width_max)):
                continue

            #縦横比による除外
            if not rect_w == 0:
                ratio = float(rect_h)/float(rect_w)
                #print(f"ratio: {ratio}")  # ratio_min, ratio_maxはこれみて調整
                if not ((ratio >= ratio_min) and (ratio <= ratio_max)):
                    continue

            #面積による除外
            mom = cv2.moments(contours[cn])
            
            #print(f"m00: {mom['m00']}")  # m00_min, m00_maxはこれみて調整
            #print(f"m10: {mom['m10']}")  # m10_min, m10_maxはこれみて調整
            if not ((mom['m00'] > m00_min) and (mom['m00'] < m00_max)):
                continue
            if not ((mom['m10'] > m10_min) and (mom['m10'] < m10_max)):
                continue
            #print(f"m00: {mom['m00']}")  # m00_min, m00_maxはこれみて調整
            #print(f"m10: {mom['m10']}")  # m10_min, m10_maxはこれみて調整

            #重心による判定
            #脚(円柱)の断面はu字なので重心のy座標が苑より下になる
            #x座標は中心近辺。中心からずれている脚は追従しない

            point_x = mom['m10']/mom['m00']
            point_y = mom['m01']/mom['m00']

            

            if center.y - point_y > diff_y:
                continue
            #print(f"diff_y: {center.y - point_y}")

            #反射強度による除外
            for i in range(rect_y,rect_y + rect_h):
                for j in range(rect_x,rect_x + rect_w):
                    #(i,j)での反射強度
                    tmp = lidar_gray_image[i,j]
                    if not tmp == 0:
                        count+=1
                        intensity += tmp

            #輪郭の平均輝度
            if not count == 0: 
                intensity /= count 
            else: 
                intensity = 0

            #反射強度による除外
            if not ((intensity > human.intensity_min) and (intensity < human.intensity_max)):
                continue

            objects[object_num].radius = radius
            objects[object_num].image_pos.x = center.x
            objects[object_num].image_pos.y = center.y
                        
            #ローカル座標系はROS似合わせて進行方向がx,左方向がy
            objects[object_num].local.y = (center.x - kImageWidth/2)/float(kMToPixel)
            objects[object_num].local.x = (center.y - kImageHeight/2)/float(kMToPixel)

            #脚候補のオブジェクトをワールド座標系に変換
            self.localToWorld(objects[object_num].local,objects[object_num].world)
            objects[object_num].setX(objects[object_num].world.x)
            objects[object_num].setY(objects[object_num].world.y)
            objects[object_num].setTheta(objects[object_num].world.theta)

            #VER2
            world_x = kMagnificationWorldImagePos * objects[object_num].world.x + kImageWidth/2 + self.displace_x
            world_y = kMagnificationWorldImagePos * objects[object_num].world.y + kImageHeight/2 + self.displace_y

            #ROSに合わせているので、行がx、列がy
            #座標の画素値が0の場合それは静止物体と判断できる
            if opening_static_object_world_pose_image[int(world_x)][int(world_y)] == 0:
                continue
            
            #脚候補オブジェクトの世界座標系での位置の画素値を0にする。
            world_pose_candidata_leg_image[int(world_x)][int(world_y)] = 0

            #脚の可能性が高い領域に対して矩形で表示
            cv2.rectangle(display_image,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),color,1)

            object_num += 1

        #VER2
        #動体を検出するための処理
        #1時刻前の画像と現在の画像の差分
        substraction_world_pose_image = ~(opening_last_world_pose_image - world_pose_candidata_leg_image)

        #動体の位置を検出
        for i in range(0,object_num):
            #脚候補のオブジェクトが動体かどうか
            objects[i].judge_dynamic = False
            world_x = int(kMagnificationWorldImagePos * objects[i].world.x + kImageWidth/2 + self.displace_x)
            world_y = int(kMagnificationWorldImagePos * objects[i].world.y + kImageHeight/2 + self.displace_y)
            
            #この座標の画素値が0であれば動体
            pixel_value = substraction_world_pose_image[int(world_x)][int(world_y)]

            if pixel_value == 0:
                #動体であればlidar_imageに描画する
                cv2.circle(lidar_image, (int(objects[i].image_pos.x), int(objects[i].image_pos.y)), 3, (200,200,0), -1,16)
                #動体の座標
                objects[i].dynamic_image_pos.x = objects[i].image_pos.x
                objects[i].dynamic_image_pos.y = objects[i].image_pos.y
                objects[i].judge_dynamic = True

        return object_num

    '''
    人の位置を推定する関数(ローカル座標系）
    object_num  : 脚候補の数
    object      : 脚候補のオブジェクト
    human_obj   : 人の位置 
    脚候補のオブジェクトから人の角度や距離を推定する
    脚の条件は2個の脚候補同士の距離が0.6[m]より離れていないこと、検出範囲の中に脚の重心が存在すること。
    移動量から脚の位置を予測
    動体を検出、予めわかっている脚に似た静止物体の位置を画像に描画
    '''
    def calcHumanPoseVer2(self,object_num, object, human_obj):
        #グローバルの宣言
        global lidar_image,static_object_world_pose_image,erode_static_object_world_pose_image
        global opening_static_object_world_pose_image,g_find_leg_radius

        '''
        人の位置推定アルゴリズム
        物体数0：ロスト
        物体数1：ロスト
        物体数2以上：2個の脚の中心、検出範囲から外れているまたは、2つの銃身が60[cm]以上離れていると除外する
        '''

        leg1_distance = 999999999.0
        leg2_distance = 999999999.0
        led1_num = 999.0
        leg1_point = Pose()
        leg2_point = Pose()
        #yamada,ver2
        last_diff_distance = 999.0

        if object_num == 0 or object_num == 1:
            human_obj.distance = 999.0
            human_obj.angle = 999.0
            human_obj.local.x = 999.0
            human_obj.local.y = 999.0
            human_obj.setX(999.0)
            human_obj.setY(999.0)
            human_obj.setTheta(999.0)
            human_obj.image_pos.x = 999.0
            human_obj.image_pos.y = 999.0
        else:
            #2個以上
            #1個目の脚を探索\
            for i in range(0,object_num):
                #オブジェクトが動体かどうか、ver
                if object[i].judge_dynamic == True:
                    #画像の中心位置からオブジェクトまでの距離
                    image1_distance = (object[i].dynamic_image_pos.x - kImageWidth/2)**2 + (object[i].dynamic_image_pos.y - kImageHeight/2)**2
                    #予測した座標からオブジェクトまでの距離
                    obj_radius = math.sqrt((object[i].dynamic_image_pos.x - human_obj.image_expect_human_x)**2 + (object[i].dynamic_image_pos.y - human_obj.image_expect_human_y)**2)
                else:
                    #動体が検出できなかった場合
                    image1_distance = (object[i].image_pos.x - kImageWidth/2)**2 + (object[i].image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].image_pos.x - human_obj.image_expect_human_x)**2 + (object[i].image_pos.y - human_obj.image_expect_human_y)**2)

                #座標が検出範囲の中にあるか
                if obj_radius < g_find_leg_radius:                
                    led1_num = i
                    leg1_distance = image1_distance
                    #動体かどうか
                    if object[i].judge_dynamic == True:
                        leg1_point.x = object[i].dynamic_image_pos.x
                        leg1_point.y = object[i].dynamic_image_pos.y
                    else:
                        leg1_point.x = object[i].image_pos.x
                        leg1_point.y = object[i].image_pos.y
                else:
                    #検出範囲外の脚候補は静止物体であると判断
                    if not object[i].judge_dynamic == True:
                        world_x = int(kMagnificationWorldImagePos * object[i].world.x + kImageWidth/2 + self.displace_x)
                        world_y = int(kMagnificationWorldImagePos * object[i].world.y + kImageHeight/2 + self.displace_y)
                        
                        #静止物体の位置の画素値をゼロにする
                        static_object_world_pose_image[world_x][world_y] = 0

                        opening_static_object_world_pose_image=cv2.morphologyEx(static_object_world_pose_image,cv2.MORPH_OPEN,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)

            #二個目の足を探索
            for i in range(0,object_num):
                #一個目の脚は飛ばす
                if i==led1_num:
                    continue

                #オブジェクトが動体かどうか
                if object[i].judge_dynamic == True:
                    image2_distance = (object[i].dynamic_image_pos.x-kImageWidth/2)**2 + (object[i].dynamic_image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].dynamic_image_pos.x - human_obj.image_expect_human_x)**2+(object[i].dynamic_image_pos.y - human_obj.image_expect_human_y)**2)

                    #leg1までの距離
                    diff_distance = math.sqrt((object[i].dynamic_image_pos.x - leg1_point.x)**2 + (object[i].dynamic_image_pos.y - leg1_point.y)**2)
                else:
                    image2_distance = (object[i].image_pos.x-kImageWidth/2)**2+(object[i].image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].image_pos.x - human_obj.image_expect_human_x)**2+(object[i].image_pos.y - human_obj.image_expect_human_y)**2)

                    #leg1までの距離
                    diff_distance = math.sqrt((object[i].image_pos.x - leg1_point.x)**2+(object[i].image_pos.y - leg1_point.y)**2)
                
                #脚候補が予測した範囲の中にあるときのみ脚と判断する
                if obj_radius < g_find_leg_radius:
                    leg2_distance = image2_distance
                    #動体かどうか
                    if object[i].judge_dynamic == True:
                        #leg1から一番近い脚候補のオブジェクトをleg2とする
                        if diff_distance < last_diff_distance:
                            leg2_point.x = object[i].dynamic_image_pos.x
                            leg2_point.y = object[i].dynamic_image_pos.y
                            last_diff_distance = diff_distance
                    else:
                        if diff_distance < last_diff_distance:
                            leg2_point.x = object[i].image_pos.x
                            leg2_point.y = object[i].image_pos.y
                            last_diff_distance = diff_distance
            
            #脚の可能性が高いものの中心の座標
            tmp_ave_x = (leg1_point.x + leg2_point.x)/2.0
            tmp_ave_y = (leg1_point.y + leg2_point.y)/2.0

            #脚の中心座標を予測
            #1時刻前の脚と座標の差分を求める
            image_diff_human_x = math.fabs(tmp_ave_x - human_obj.last_human_x)
            image_diff_human_y = math.fabs(tmp_ave_y - human_obj.last_human_y)
            
            #求めた差分から人の位置を予測し、次の検出範囲の中心とする
            human_obj.image_expect_human_x = (tmp_ave_x + image_diff_human_x)
            human_obj.image_expect_human_y = (tmp_ave_y + image_diff_human_y)

            #矩形
            tmp_radius = math.sqrt((tmp_ave_x - human_obj.image_expect_human_x)**2 + (tmp_ave_y - human_obj.image_expect_human_y)**2)

            #脚候補の中心が検出範囲尾の中にあるか
            human_pos_judge = ((math.fabs(tmp_ave_x - human_obj.image_expect_human_x) < g_find_leg_radius) and (math.fabs(tmp_ave_y - human_obj.image_expect_human_y) < g_find_leg_radius))

            #片足間の重心の距離
            d = math.sqrt((leg1_point.x - leg2_point.x)**2 + (leg1_point.y - leg2_point.y)**2) /float(kMToPixel)

            #脚の位置からの矩形を描写
            cv2.rectangle(lidar_image,(int(human_obj.image_expect_human_x - g_find_leg_radius),int(human_obj.image_expect_human_y -  g_find_leg_radius)),(int(human_obj.image_expect_human_x + g_find_leg_radius),int(human_obj.image_expect_human_y + g_find_leg_radius)),(0,255,0),2)         
            
            #脚と判断したときの処理
            if (d < kLegBetweenDistance) and (human_pos_judge):
                #画像中心から脚までの距離[m]
                human_obj.distance = (math.sqrt(leg1_distance) + math.sqrt(leg2_distance))/(2*kMToPixel)
                human_obj.angle = (math.atan2(leg1_point.x - kImageWidth/2,kImageHeight/2 - leg1_point.y) + (math.atan2(leg2_point.x - kImageWidth/2,kImageHeight/2 - leg2_point.y)))/2.0

                #現在の脚の中心座標
                ave_x =(leg1_point.x + leg2_point.x)/2.0
                ave_y =(leg1_point.y + leg2_point.y)/2.0

                #脚を見失っていないときは一時刻前の座標に現在の座標を代入
                if (not ave_x == 999.0) and (not ave_y == 999.0):
                    human_obj.last_human_x = ave_x
                    human_obj.last_human_y = ave_y

                #人の位置のx座標、見失った場合は1時刻前の座標を使用
                if human_obj.image_expect_human_x == 0.0:
                    human_obj.image_pos.x = human_obj.last_human_x
                else:
                    human_obj.image_pos.x = human_obj.image_expect_human_x
                #人の位置のy座標、見失った場合は1時刻前の座標を使用
                if human_obj.image_expect_human_y == 0.0:
                    human_obj.image_pos.y = human_obj.last_human_y
                else:
                    human_obj.image_pos.y = human_obj.image_expect_human_y

            else:
                self.find_fuman_msg.data = False
                self.find_human_pub.publish(self.find_fuman_msg)

                human_obj.distance = 999.0
                human_obj.angle = 999.0
                human_obj.local.x = 999.0
                human_obj.local.y = 999.0
                human_obj.setX(999.0)
                human_obj.setY(999.0)
                human_obj.setTheta(999.0)
                human_obj.image_pos.x = 999.0
                human_obj.image_pos.y = 999.0

    '''
    ロボットの状態をチェックする関数
    theta   : 角度
    DENGER  : 障害物あり
    SAFE    : 障害物なし

    危険領域内に障害物が歩かないかをチェックする
    危険領域はstop_width，stop_distanceの値で決めている。
    '''

    def checkCondition(self,theta):
        #走査線の半分540
        center = self.dataCount/2
        #正面180度のみの走査線数720
        search_lines = 1080.0*(180.0/270.0)
        x = y = angle = 0.0
        #止まるときの範囲、lidarの中心位置から
        stop_width = 0.25
        stop_distance = 0.30

        j = 0

        for i in range(int(center - search_lines/2) , int(center + search_lines/2)+1):

            #1.5は180度を270度に対応させるため、hokuyoLiDARの最大測定角度は270
            angle = j*(1.5*math.pi)/1080
            j+=1
            #角度theta[rad]にそうそうする走査線の数だけ足す
            k = int(i + theta*1080/(1.5*math.pi))
            if (k < center - search_lines/2) or (k > center + search_lines/2):
                continue

            #検出したときの座標
            x = self.laser_distance[k] * math.cos(angle)
            y = self.laser_distance[k] * math.sin(angle)

            #障害物が停止範囲に入った場合
            if (math.fabs(x) <= stop_width) and (math.fabs(y) <= stop_distance):
                #衝突する可能性が高い
                return DANGER

        return SARE

    '''
    衝突を検出する関数
    avoid_angle : 障害物のいる方向
    SAFE        : 安全
    DENGER      : 危険
    左右に5度ずつの角度をcheckConditionに渡し、障害物がないかチェックする
    '''

    def checkCollision(self):
        for i in range(0,90,5):
            for j in range(-1,1):
                condition = 0
                angle = i*j
                avoid_angle = 0
                condition = self.checkCondition(DEG2RAD(angle))

                if condition == SARE:
                    #LiDARのスキャン方向は半時計回りなのでマイナスをかける
                    avoid_angle = -angle #この値を使えば障害物回避が可能
                    return SARE,avoid_angle
                elif condition ==  DANGER:
                    avoid_angle = -angle
                    return DANGER,avoid_angle
    

    '''
    脚の位置から人を追従する関数
    input_image : LiDAR画像
    calcHumanPoseから人の位置を推定し、並進や回転の制御を行う
    人を見失いかけたときは検出範囲を拡大する
    '''

    def followHuman(self,input_image,moveble = True):
        #グローバル使用宣言
        global lidar_image,g_find_leg_radius
        global human
        
        # chaserが動作しているかパブリッシュ by Kanazawa(2023)
        self.chaser_check_msg.data = moveble
        if moveble == False:
            self.move(0.0, 0.0)
            self.chaser_check.publish(self.chaser_check_msg)
            return
        self.chaser_check.publish(self.chaser_check_msg)

        #脚候補
        obj = [Object() for i in range(0,100)]
        #オブジェクト数
        obj_num = 0

        #脚候補を見つけるために元画像で連続領域を探索
        #脚候補Wクォ見つけてその数を返す
        #長ズボンの際は0次モーメント(輪郭面積)を40,半ズボンなら34
        obj_num = self.findLegs(input_image,obj, lidar_image, red,6,30, 4, 30, 0.7, 3.0, 30, 100, 10000, 25000, 1.5, 0.3)

        #人間の位置と方向を計算(ローカル座標系)
        self.calcHumanPoseVer2(obj_num,obj,human)

        #画像表示のために1ミリ秒待つ
        cv2.waitKey(1)

        #見つけた人の位置に丸を描写
        cv2.circle(lidar_image,(int(human.image_pos.x),int(human.image_pos.y)),5,green,2)
        self.find_fuman_msg.data = True
        self.find_human_pub.publish(self.find_fuman_msg)

        #この回数だけ連続して失敗すると検出範囲を拡大
        lost_count_max = 1

        #人を見失ったとき999
        if human.distance == 999.0:
            self.lost_count += 1
        else: 
            #見失っていない
            self.human_lost = False
            self.lost_count = 0

            #検出範囲をもとの大きさにする
            g_find_leg_radius = kOrignalRadius

        #lost_count_max会連続で発見できなかった際、検出範囲を拡大する
        if self.lost_count >= lost_count_max:
            self.human_lost = True
            g_find_leg_radius = kExtendRadius

        #人を見失ったとき、一時刻前の値を使う
        if self.human_lost == True:
            human.distance = human.last_distance
            human.angle = human.last_angle

        #追跡中の並進の制御
        if human.distance == 999.0:#人を完全に見失ったとき
            #減速して停止
            self.reduceSpeed(human)
            #静止物体の位置を初期化
            self.defaultPos(human)

        elif (human.distance >= kFollowMinDistance) and (human.distance <= kFollowMaxDistance): #人が追跡範囲内にいる場合
            if self.lost_count < 15: #0.45秒以上見失わなければ追従する
                tmp_speed = self.followLinearSpeed(human)
                #一時刻前の速度
                human.last_linear_speed = tmp_speed
                self.setLinearSpeed(tmp_speed)
            else:
                #ふらついて見失う場合が多いので、見失いそうなときは角速度を0[rad/s]に設定する
                self.setAngularSpeed(0)
                #人を完全に見失ったら初期化する
                if self.lost_count >= kLostTime:
                    self.defaultPos(human)
                    human.distance = 999.0
                    human.angle = 999.0
                    self.get_logger().info("I lost the human")

        else: #人が追跡対象外に出たら
            #減速して停止
            self.reduceSpeed(human)
            #静止物体の初期化
            self.defaultPos(human)

        #追跡中の回転制御
        #5度以内のときは回転しない
        if (human.angle == 999.0) or (human.last_angle == 999.0):
            self.setAngularSpeed(0.0)            
        elif math.fabs(human.angle) > DEG2RAD(5.0):
            tmp_speed = self.getAngularSpeed()
            tmp_speed += -kKp * human.angle - kKd * (human.angle - human.last_angle)
            self.setAngularSpeed(tmp_speed)

        #速度の上限と下限を設定
        if self.getAngularSpeed() > kTurnMaxSpeed:
            self.setAngularSpeed(kTurnMaxSpeed)
        if self.getAngularSpeed() < -kTurnMaxSpeed:
            self.setAngularSpeed(-kTurnMaxSpeed)

        #衝突検出
        collision,avoid_angle = self.checkCollision()

        if collision == DANGER:
            #衝突する可能性が高いので、下がって障害物のない方向に方向転換する
            turn_speed = 0.2
            if DEG2RAD(avoid_angle) > 0:
                self.move(0,self.getAngularSpeed() - turn_speed)
            else:
                self.move(0,self.getAngularSpeed() + turn_speed)
            time.sleep(0.02)
            #self.get_logger().info("DANGER")
        else: #衝突する可能性は低いので障害物と反対方向へ少し向きを変更
            self.move(self.getLinearSpeed(),self.getAngularSpeed())
            #self.get_logger().info("SAFE")

        human.last_distance = human.distance
        human.last_angle = human.angle

    '''
    LiDAR画像の処理をする関数
    lIDAR画像に縮小処理をし、ノイズを除去する
    '''

    def prepWindow(self):
        #グローバル関数
        global lidar_gray_image
        global lidar_image,lidar_erode_image
        #グレースケールに変換する
        ret,lidar_bin_image = cv2.threshold(lidar_gray_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #反転
        lidar_bin_image = ~(lidar_bin_image)
        #縮小、ノイズ除去
        lidar_erode_image = cv2.erode(lidar_bin_image,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)
        #カラー画像に変換
        lidar_color_image = cv2.cvtColor(lidar_gray_image,cv2.COLOR_GRAY2BGR)
        lidar_image = lidar_color_image

    '''
    画像を表示する関数
    '''

    def showWindow(self):
        #グローバル宣言
        global lidar_image

        cv2.namedWindow("Map", cv2.WINDOW_AUTOSIZE)
        dst_img = ~(lidar_image)
        #dst_img = lidar_image

        #画像表示
        cv2.imshow("Map",dst_img)

    def warp_angle(angle):
        wrapped = 0
        if (angle <= math.pi) and (angle >= -1.0*math.pi):
            wrapped = angle
        elif angle < 0.0:
            wrapple = math.fmod(angle - math.pi,2.0*math.pi)+math.pi
        else:
            wrapple = math.fmod(angle + math.pi,2.0*math.pi)-math.pi

        return wrapped

    '''
    オドメトリのコールバック関数
    位置や角度をx,y,thetaに代入
    '''

    def odomCallback(self,msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        rpy = euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
        self.robot_theta = rpy[2]

    '''
    #followのstart,stop用のコールバック数
    '''

    def followHumanCallback(self,msg):
        if msg.data == "start":
            self.follow_command = "start"
        elif msg.data == "stop":
            self.follow_command = "stop"


def main():
    #ROSの初期化
    rclpy.init()

    robot = Robot()

    #パラメータ表示
    robot.welcomeMessage()

    #33Hzのタイマー
    loop_rate = 1/33
    sum_time = 0.0
    cut = 0

    loop = 0

    time_counter = time.time()

    # Trueでcmd_velのパブリッシュをとめる
    stop_flg = False

    while rclpy.ok():
        rclpy.spin_once(robot, timeout_sec=0.1)
        robot.laser_cycle()
        if loop < 10:
            time.sleep(loop_rate)
            cut+=1
            loop+=1
            robot.get_logger().info(f"time = {time.time() - time_counter}")
            continue
 
        if robot.follow_command == "start":
            stop_flg = False
            robot.followHuman(lidar_erode_image,True)
            robot.showWindow()
        elif robot.follow_command == "stop" and stop_flg:
            pass
        elif robot.follow_command == "stop":
            stop_flg = True
            robot.followHuman(lidar_erode_image,False)
        

        time.sleep(loop_rate)
        cut+=1
