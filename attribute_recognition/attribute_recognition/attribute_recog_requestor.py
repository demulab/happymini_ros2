import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from happymini_msgs.srv import StringCommand, SpeechToText, NameDetect, TextToSpeech, AttributeRecognition, DetectPerson
from happymini_msgs.msg import DetectionResult
import cv2
from cv_bridge import CvBridge

class AttributeRecog(Node):
    def __init__(self):
        super().__init__("attrib")
        self.node = rclpy.create_node('attrib_recog')
        self.subscription = self.create_subscription(
            DetectionResult,
            "/fmm/detection_results",
            self.detection_callback,
            qos_profile_sensor_data
        )
        self.__client = self.node.create_client(AttributeRecognition, '/fmm_attrib_service/recognize')
        self.__req = None
        self.future = None 
        self.bridge = CvBridge()
        self.count = 0
        
    def detection_callback(self, msg : DetectionResult):
        img = self.findPerson(msg)
        self.count += 1
        cv2.imwrite("/home/demulab/data/{0}.png".format(self.count), self.bridge.imgmsg_to_cv2(img, "bgr8"))
        result = self.execute(img)
        print(result)

    def findPerson(self, msg : DetectionResult) -> Image:
        likelihood = 0
        img = Image()
        for i in range(len(msg.tags)):
            if msg.tags[i] == "person" and msg.likelihood[i] > likelihood:
                img = msg.images[i]
        
        return img


    def execute(self, img : Image) -> str:
        self.__req = AttributeRecognition.Request()
        self.__req.input = img
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
    node = AttributeRecog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
