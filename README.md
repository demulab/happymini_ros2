<<<<<<< HEAD
# happymini_ros2
# 開発中
## 設定
以下3種のウエイポイントを登録する
1. fmm_Operator : オペレータ開始位置
2. fmm_find1 : 一人目の探索位置
3. fmm_find2 : 二人目の探索位置

### マップを新規作成する場合
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

```bash
ros2 run happymini_navigation set_location.py
```
その後、サービス呼び出しでウェイポイントを登録する。

### 既存のマップにウェイポイントを追加する場合
```bash
ros2 launch happymini_navigation navigation2.launch.py
```
```bash
ros2 run happymini_navigation set_location.py
```
その後、サービス呼び出しでウェイポイントを登録する。

=======
# Carry My Luggage 50点
>>>>>>> kanazawa-devel
## 実行
turtlebot3とlidarのポート権限付与
```bash
sudo chmod 666 /dev/ttyACM*
```
<<<<<<< HEAD
=======

>>>>>>> kanazawa-devel
bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
<<<<<<< HEAD
navigation
```bash
ros2 launch happymini_navigation navigation2.launch.py
```

fmm_speech_service
```bash
ros2 run happymini_voice fmm_speech_service
```
realsenseの起動
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```
魚眼カメラの起動
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args --param video_device:=/dev/video2
```

属性認識
```bash
ros2 run attribute_recognition attribute_recog_node
```

人認識、距離推定
```bash
ros2 run yolov5_ros2 object_detection_tf
```

人認識②
```bash
ros2 run yolov5_ros2 person_detector
```
※将来的に人認識はこれに変更する予定

approach_person
```bash
ros2 run approach_person approach_person
=======

open_manipulator_x
```bash
ros2 launch open_manipulator_x_description open_manipulator_x.launch.py
```

grasp_bag
```bash
ros2 launch grasp_bag grasp_bag.launch.py
```

detect_pose
```bash
ros2 run detect_pose detect_left_right
```

chaser_python
```bash
ros2 run chaser_python chaser_python
```

happymini_voice
```bash
ros2 run happymini_voice speech_to_text 
ros2 run happymini_voice tts_coqui 
>>>>>>> kanazawa-devel
```

**マスタープログラム**
```bash
<<<<<<< HEAD
ros2 run rcj_2023_master find_my_mates_2023
=======
ros2 run rcfrance_2023_master carry_my_luggage_france2023
```
>>>>>>> kanazawa-devel

**マイクの設定**
![Screenshot from 2023-06-18 18-03-12](https://github.com/demulab/happymini_ros2/assets/121123734/d13841d3-dc5b-4f20-9a65-98fabf300b51)

**魚眼カメラ**
USBを挿す

![image_167837697](https://github.com/demulab/happymini_ros2/assets/121123734/febeb32c-a84b-470f-bb9f-b49f37a2fb64)

