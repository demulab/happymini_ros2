import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from rclpy.action import ActionClient
#from happymini_msgs.srv import BagLocalization
#from happymini_msgs.action import GraspBag
import time
#from happymini_manipulation.motor_controller import JointController
from happymini_msgs.srv import StringCommand, TextToSpeech, AttributeRecognition, DetectPerson
from happymini_navigation.navi_location import WayPointNavi
import pyttsx3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

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

    def execute(self, waypoint :str):
        #self.wp_node.set_params()
        self.wp_node.navigation_execute(waypoint)
        time.sleep(2)

    def execute2(self):
        self.wp_node.navigation_execute('fmm_Operator')
        time.sleep(2)
    
    def execute3(self):
        self.wp_node.navigation_execute('fmm_fimd2')
        time.sleep(2)

class HitoSekkin(Node):
    def __init__(self):
        super().__init__('test_hitosekkin')
        self.app_srv = self.create_client(TextToSpeech, 'app')
        while not self.app_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")
        self.app_srv_req = TextToSpeech.Request()
        #self.pub = self.create_publisher(String, 'master', 10)

    def execute(self):
        self.app_srv_req.text = 'start'
        
        app_srv_future = self.app_srv.call_async(self.app_srv_req)
        while not app_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            print('while')
        print('out while')
        if app_srv_future.result() is not None:
            app_srv_result = app_srv_future.result().result
            print(app_srv_result)
            return app_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

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

        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info('応答: ' + response.answer)
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        return response.answer_name

class AttributeRecog(Node):
    def __init__(self):
        super().__init__("attrib")
        self.node = rclpy.create_node('attrib_recog')
        self.__client = self.node.create_client(AttributeRecognition, '/fmm_attrib_service/recognize')
        self.__req = None
        self.future = None 

    def execute(self) -> str:
        self.__req = AttributeRecognition.Request()
        self.future = self.__client.call_async(self.__req)
        rclpy.spin_until_future_complete(self.node, self.future)

        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.future.result() is not None:
            response = self.future.result()
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        return response.result

class PersonDetector(Node):
    def __init__(self):
        super().__init__("person")
        self.node = rclpy.create_node('person_recog')
        self.__client = self.node.create_client(DetectPerson, '/fmm_person_service/detect')
        self.__req = None
        self.future = None 

    def execute(self) -> Image:
        self.__req = DetectPerson.Request()
        self.future = self.__client.call_async(self.__req)
        rclpy.spin_until_future_complete(self.node, self.future)

        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.future.result() is not None:
            response = self.future.result()
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        return response.result    

    
def main():
    rclpy.init()
    synthesis2("Start, find my mates.")
    nb = Navigation()
    hi = HitoSekkin()
    sp = Speech()
    at = AttributeRecog()
    per = PersonDetector()
    
    try:
        #1人目
        nb.execute('fmm_find')
        time.sleep(1.0)
        hi.execute()
        name = sp.execute()
        img = per.execute()
        nb.execute('fmm_Operator')
        time.sleep(1.0)
        sp.execute()
        synthesis2("Name is " + name)
        attribute_sentence = at.execute()
        synthesis2(attribute_sentence)
        
        ##2人目
        nb.execute('fmm_find2')
        time.sleep(1.0)
        hi.execute()
        name = sp.execute()
        img = per.execute()
        nb.execute('fmm_Operator')
        time.sleep(1.0)
        sp.execute()
        synthesis2("Name is " + name)
        attribute_sentence = at.execute()
        synthesis2(attribute_sentence)

        # 3人目今後やる

        synthesis2("Finish, find my mates.")
    except KeyboardInterrupt:
        pass
    finally:
        #cd.destroy_node()
        rclpy.shutdown()
