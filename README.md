# Find My Mates 110点
## 実行
turtlebot3とlidarのポート権限付与
```bash
sudo chmod 666 /dev/ttyACM*
```
bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
fmm_speech_service
```bash
ros2 run happymini_voice fmm_speech_service
```
realsenseの起動
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

人認識、距離推定
```bash
ros2 run yolov5_ros2 object_detection_tf
```

approach_person
```bash
ros2 run approach_person approach_person
```

**マスタープログラム**
```bash
ros2 run rcj_2023_master find_my_mates_2023
```
