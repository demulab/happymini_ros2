import rclpy
from rclpy.node import Node
from happymini_msgs.srv import SpeechToText, NameDetect

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.stt_srv = self.create_client(SpeechToText, 'stt')
        self.nd_srv = self.create_client(NameDetect, 'nd')
        while not self.stt_srv.wait_for_service(timeout_sec=1.0) and self.nd_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Message is not here ...")
        self.stt_srv_req = SpeechToText.Request()
        self.nd_srv_req = NameDetect.Request()

    def stt_send_request(self, tex='start'):
        stt_srv_result = 'None'
        self.stt_srv_req.tex = 'start'

        stt_srv_future = self.stt_srv.call_async(self.stt_srv_req)
        while not stt_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if stt_srv_future.result() is not None:
            stt_srv_result = stt_srv_future.result().result
            print(stt_srv_result)
            return stt_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

    def nd_send_request(self, text=None):
        nd_srv_result = 'None'
        self.nd_srv_req.text = text

        nd_srv_future = self.nd_srv.call_async(self.nd_srv_req)
        while not nd_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if nd_srv_future.result() is not None:
            nd_srv_result = nd_srv_future.result().result
            print(nd_srv_result)
            return nd_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None




def main():
    rclpy.init()
    tc = TestClient()
    try:
        stt = tc.stt_send_request()
        tc.nd_send_request(stt)
        rclpy.spin_once(tc)
    except KeyboardInterrupt:
        pass
    tc.destroy_node()
    rclpy.shutdown()
    print('end')  
