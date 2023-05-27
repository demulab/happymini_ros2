# Carry My Luggage 50点
## 実行
turtlebot3とlidarのポート権限付与
```bash
sudo chmod 666 /dev/ttyACM*
```

bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

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
```

**マスタープログラム**
```bash
ros2 run rcfrance_2023_master carry_my_luggage_france2023
```
