import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from rclpy.action import ActionClient
#from happymini_msgs.srv import BagLocalization
#from happymini_msgs.action import GraspBag
import time
#from happymini_manipulation.motor_controller import JointController
from happymini_msgs.srv import StringCommand, SpeechToText, TextToSpeech, SetimentAnalysis
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import cv2

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.stt_srv_req = SpeechToText.Request()
        self.sa_srv_req = SetimentAnalysis.Request()
        self.tts_srv_req = TextToSpeech.Request()

        self.stt_srv = self.create_client(SpeechToText, 'stt')
        self.sa_srv = self.create_client(SetimentAnalysis, 'final/setiment_analysis')
        self.tts_srv = self.create_client(TextToSpeech, 'tts')
        while not self.stt_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")


    def stt_send_request(self, cmd='start'):
        stt_srv_result = 'None'
        stt_srv_req = SpeechToText.Request()
        stt_srv_req.cmd = cmd
        self.stt_srv = self.create_client(SpeechToText, 'stt')

        stt_srv_future = self.stt_srv.call_async(stt_srv_req)
        while not stt_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if stt_srv_future.result() is not None:
            stt_srv_result = stt_srv_future.result().result
            print(stt_srv_result)
            return stt_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None
    
    def tts_send_request(self, text=None):
        tts_srv_result = 'None'
        self.tts_srv_req.text = text

        tts_srv_future = self.tts_srv.call_async(self.tts_srv_req)
        while not tts_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if tts_srv_future.result() is not None:
            tts_srv_result = tts_srv_future.result().result
            print(tts_srv_result)
            return tts_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

    def sa_send_request(self, text=None):
        sa_srv_result = 'None'
        self.sa_srv_req.word = text

        sa_srv_future = self.sa_srv.call_async(self.sa_srv_req)
        while not sa_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if sa_srv_future.result() is not None:
            sa_srv_result = sa_srv_future.result()
            print(sa_srv_result)
            return sa_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

       
   

    
def main():
    rclpy.init()
    tc = TestClient()
    try:
        happy_score = 0
        bad_score = 0
        surprised_score = 0
        while bad_score < 5:
            print("bad score :", bad_score)
            word = tc.stt_send_request()
            emotion = tc.sa_send_request(word)

            max_score = 0
            max_idx = 0
            for i in range(len(emotion.emotion)):
                if max_score < emotion.likelihood[i]:
                    max_score = emotion.likelihood[i]
                    max_idx = i
            
            if emotion.emotion[max_idx] in ["anger", "disgust"]:
                bad_score += 1
                tc.tts_send_request(f"Detected negative feeling for {bad_score} time.")
                if bad_score == 2:
                    tc.tts_send_request("The mood is getting bad.")
                elif bad_score == 4:
                    tc.tts_send_request("This is going to be a fight.")
            elif emotion.emotion[max_idx] in ["surprise"]:
                surprised_score += 1
                tc.tts_send_request("There seems to be a trouble happening.")
            time.sleep(1)

        tc.tts_send_request("Hello, I am Happy Mini.")
        tc.tts_send_request("Let's stop the fight.")
        tc.tts_send_request("I am going to make you happy.")        
        
    except KeyboardInterrupt:
        pass
    finally:
        #cd.destroy_node()
        rclpy.shutdown()
