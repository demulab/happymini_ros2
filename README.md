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

## 実行
turtlebot3とlidarのポート権限付与
```bash
sudo chmod 666 /dev/ttyACM*
```
bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
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

人認識3
```bash  
ros2 run yolov5_ros2 person_in_area
```

approach_person
```bash
ros2 run approach_person approach_person
```

**マスタープログラム**
```bash
ros2 run rcj_2023_master find_my_mates_2023
