import rclpy
from rclpy.node import Node
from happymini_msgs.srv import SpeechToText, NameDetect, TextToSpeech

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.stt_srv = self.create_client(SpeechToText, 'stt')
        self.nd_srv = self.create_client(NameDetect, 'nd')
        self.tts_srv = self.create_client(TextToSpeech, 'tts')
        #self.pia_srv = self.create_client(PersonInArea, 'srv_personinarea')
        while not self.stt_srv.wait_for_service(timeout_sec=1.0) and self.nd_srv.wait_for_service(timeout_sec=1.0) and self.nd_srv.wait_for_service(timeout_sec=1.0) and self.nd_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")
        self.stt_srv_req = SpeechToText.Request()
        self.nd_srv_req = NameDetect.Request()
        self.tts_srv_req = TextToSpeech.Request()

#    def stt_send_request(self, cmd='start'):
#        stt_srv_result = 'None'
#        self.stt_srv_req.cmd = cmd
#
#        stt_srv_future = self.stt_srv.call_async(self.stt_srv_req)
#        while not stt_srv_future.done() and rclpy.ok():
#            rclpy.spin_once(self, timeout_sec=0.1)
#        if stt_srv_future.result() is not None:
#            stt_srv_result = stt_srv_future.result().result
#            print(stt_srv_result)
#            return stt_srv_result
#        else:
#            self.get_logger().info(f"Service call failed")
#            return None
#
#    def nd_send_request(self, text=None):
#        nd_srv_result = 'None'
#        self.nd_srv_req.text = text
#
#        nd_srv_future = self.nd_srv.call_async(self.nd_srv_req)
#        while not nd_srv_future.done() and rclpy.ok():
#            rclpy.spin_once(self, timeout_sec=0.1)
#        if nd_srv_future.result() is not None:
#            nd_srv_result = nd_srv_future.result().result
#            print(nd_srv_result)
#            return nd_srv_result
#        else:
#            self.get_logger().info(f"Service call failed")
#            return None
#
#    def tts_send_request(self, text=None):
#        tts_srv_result = 'None'
#        self.tts_srv_req.text = 'You are' + text
#
#        tts_srv_future = self.tts_srv.call_async(self.tts_srv_req)
#        while not tts_srv_future.done() and rclpy.ok():
#            rclpy.spin_once(self, timeout_sec=0.1)
#        if tts_srv_future.result() is not None:
#            tts_srv_result = tts_srv_future.result().result
#            print(tts_srv_result)
#            return tts_srv_result
#        else:
#            self.get_logger().info(f"Service call failed")
#            return None

#    def pia_send_request(self, cmd='start'):
#        pia_srv_result = 'None'
#        self.pia_srv_req.cmd = cmd
#
#        pia_srv_future = self.pia_srv.call_async(self.pia_srv_req)
#        while not pia_srv_future.done() and rclpy.ok():
#            rclpy.spin_once(self, timeout_sec=0.1)
#        if pia_srv_future.result() is not None:
#            pia_srv_result = pia_srv_future.result().result
#            print(pia_srv_result)
#            return pia_srv_result
#        else:
#            self.get_logger().info(f"Service call failed")
#            return None




def main():
    rclpy.init()
    tc = TestClient()
    try:
        #stt = tc.stt_send_request()
        #nd = tc.nd_send_request(stt)
        #tc.tts_send_request(nd)
        #tc.pia_send_request()
        rclpy.spin_once(tc)
    except KeyboardInterrupt:
        pass
    tc.destroy_node()
    rclpy.shutdown()
    print('end')  
