import rclpy
from transformers import pipeline
from rclpy.node import Node
from happymini_msgs.srv import SetimentAnalysis

class SetimentDetector(Node):
    def __init__(self):
        super().__init__("setiment_detector_node")
        self.classifier = pipeline("text-classification", model="j-hartmann/emotion-english-distilroberta-base", return_all_scores=True)
        self.service = self.create_service(SetimentAnalysis, "/final/setiment_analysis", self.setimentAnalysis)

    def setimentAnalysis(self, request, response):
        emotion = self.classifier(request.word)[0]
        print(emotion)
        for i in range(len(emotion)):
            response.emotion.append(emotion[i]["label"])
            response.likelihood.append(emotion[i]["score"])

        return response
    

def main(args=None):
    rclpy.init(args=args)
    setiment_analysis_node = SetimentDetector()
    rclpy.spin(setiment_analysis_node)
    rclpy.shutdown()