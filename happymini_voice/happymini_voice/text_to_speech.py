import time
import pyttsx3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from happymini_msgs.srv import TextToSpeech


class TextToSpeechServer(Node):
    def __init__(self):
        super().__init__('text_to_speech')
        self.create_service(TextToSpeech, 'tts', self.speech)
        # Params from launch
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('rate', Parameter.Type.INTEGER),
                    ('volume', Parameter.Type.DOUBLE),
                    ('voice_type', Parameter.Type.INTEGER)])
        # Module
        self.engine = pyttsx3.init()
        self.get_logger().info("Ready to set /text_to_speech server")

    def set_property(self):
        # パラメータ取得
        rate, volume, voice_type = self.get_parameters(['rate', 'volume', 'voice_type'])
        self.get_logger().info(f"rate: {rate.value}")
        self.get_logger().info(f"volume: {volume.value}")
        self.get_logger().info(f"voice_type: {voice_type.value}")
        # 音声のスピード調整
        self.engine.setProperty('rate', rate.value)
        # 音量調整
        self.engine.setProperty('volume', volume.value)  # デフォルトは1.0
        # 音声のタイプの設定
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[voice_type.value].id)

    def speech(self, srv_req, srv_res):
        # プロパティ
        self.set_property()
        # テキストをセット
        self.engine.say(srv_req.text)
        # 実行
        self.engine.runAndWait()
        srv_res.result = True
        return srv_res


def main():
    rclpy.init()
    ttss = TextToSpeechServer()
    try:
        rclpy.spin(ttss)
    except KeyboardInterrupt:
        pass
    ttss.destroy_node()
    rclpy.shutdown()
