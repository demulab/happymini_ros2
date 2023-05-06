import rclpy
from rclpy.node import Node
#from rclpy.action import ActionClient
from std_msgs.msg import String
#from happymini_msgs.srv import BagLocalization
#from happymini_msgs.action import GraspBag
import time
#from happymini_manipulation.motor_controller import JointController

from airobot_interfaces.srv import StringCommand
from happymini_navigation.navi_location import WayPointNavi
import playsound
import wave
import pyaudio
import whisper
import sys
import pyttsx3

def synthesis2(text = None):
    # 音声合成を行うことをloggerで表示します．
    print(f'音声合成を実行します')
    print(f'発話内容は "{text}"')

    # 音声合成エンジンを初期化します．
    engine = pyttsx3.init()

    # 言語を設定します．
    lang = "en-US"
    engine.setProperty('voice', lang)
    # rateは、1分あたりの単語数で表した発話速度。基本は、200です。
    rate = engine.getProperty("rate")
    engine.setProperty("rate",160)

    # ボリュームは、0.0~1.0の間で設定します。
    volume = engine.getProperty('volume')
    engine.setProperty('volume',1.0)

    # テキストを音声に合成します．
    engine.say(text)

    engine.runAndWait()

        
class Navigation(Node):
    def __init__(self):
        super().__init__('test_navigation')
        self.wp_node = WayPointNavi()

    def execute(self):
        #self.wp_node.set_params()
        self.wp_node.navigation_execute('start_cml')
        time.sleep(2)

    def execute2(self):
        self.wp_node.navigation_execute('start_fmm')
        time.sleep(2)
    
    def execute3(self):
        self.wp_node.navigation_execute('goal_car1')
        time.sleep(2)

class HitoSekkin(Node):
    def __init__(self):
        super().__init__('test_hitosekkin')
        self.pub = self.create_publisher(String, 'master', 10)

    def execute(self):
        msg = String()
        msg.data = 'start'
        self.pub.publish(msg)

class Speech(Node):
    def __init__(self):
        super().__init__('test_speech') 
        self.node = rclpy.create_node('fmm_client')
        self.client = self.node.create_client(StringCommand, 'fmm_speech_service/wake_up')
        self.req = None
        self.future = None
        
    def execute(self):
        time.sleep(1.0)
        self.req = StringCommand.Request()
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info('応答: ' + response.answer)
        else:
           self.get_logger().info('サービスが応答しませんでした。')
            
    
def main():
    rclpy.init()
    synthesis2("Start, find my mates.")
    nb = Navigation()
    hi = HitoSekkin()
    sp = Speech()
    try:
        #nb.execute()
        #hi.execute()
        #sp.execute()
        #nb.execute2()
        sp.execute()
        synthesis2("Finish, find my mates.")
    except KeyboardInterrupt:
        pass
    finally:
        #cd.destroy_node()
        rclpy.shutdown()
