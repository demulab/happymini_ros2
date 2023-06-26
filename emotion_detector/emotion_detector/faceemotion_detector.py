import rclpy
from deepface import DeepFace
from rclpy.node import Node
from happymini_msgs.srv import FaceEmotion
from cv_bridge import CvBridge
from PIL import Image as PILImage

class FaceEmotionDetector(Node):
    def __init__(self):
        super().__init__("face_emotion_detector_node")
        self.__facedetect_backends = [
            'opencv', 
            'ssd', 
            'dlib', 
            'mtcnn', 
            'retinaface', 
            'mediapipe'
        ]
        self.service = self.create_service(FaceEmotion, "/final/face_emotion", self.detectEmotion)
        self.bridge = CvBridge()

    def detectEmotion(self, request, response):
        cv_img = self.bridge.imgmsg_to_cv2(request.image)
        try:
            demographies = DeepFace.analyze(cv_img, 
                detector_backend = self.__facedetect_backends[3]
            )
            emotion = demographies[0]

            for key, value in emotion["emotion"].items():
                response.emotion.append(key)
                response.likelihood.append(value)
            return response
        except ValueError:
            response.emotion.append("neutral")
            response.likelihood.append(0)
            return response
    
def main(args=None):
    rclpy.init(args=args)
    face_emotion_node = FaceEmotionDetector()
    rclpy.spin(face_emotion_node)
    rclpy.shutdown()