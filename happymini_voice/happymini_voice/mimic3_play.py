import os
import wave
import yaml
from ovos_tts_plugin_mimic3 import Mimic3TTSPlugin
from playsound import playsound
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# Custom msgs
from happymini_msgs.srv import TextToSpeech


class Mimic3PlayServer(Node):
    def __init__(self):
        super().__init__('mimic3_play_node')
        # Service
        self.create_service(TextToSpeech, 'tts', self.speak)
        self.get_logger().info("Ready to set /mimic3_play_server")
        # Set params
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('voice', Parameter.Type.STRING),
                    ('speaker', Parameter.Type.STRING),
                    ('length_scale', Parameter.Type.DOUBLE),
                    ('noise_scale', Parameter.Type.DOUBLE),
                    ('noise_w', Parameter.Type.DOUBLE)])
        # Path
        self.save_path = None
        self.pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.wav_path = os.path.join(
                self.pkg_path,
                'config',
                'mimic3')
        # Mimic3 Plugin
        self.mimic3_plug = None

    def get_params(self):
        # Get params
        voice = self.get_parameter('voice').value
        speaker = self.get_parameter('speaker').value
        length_scale = self.get_parameter('length_scale').value
        noise_scale = self.get_parameter('noise_scale').value
        noise_w = self.get_parameter('noise_w').value
        # Mimic3 Plugin
        cfg = {'voice':voice, 'speaker':speaker, 'length_scale':length_scale, 'noise_scale':noise_scale, 'noise_w':noise_w}
        self.mimic3_plug = Mimic3TTSPlugin(config=cfg)
        self.get_logger().info(f"Get params: {cfg}")

    def text_to_wav(self, text, file_name):
        self.get_params()
        # 「file_path」に「.wav」がなかったらつける
        dot_wav = '.wav'
        if '.wav' in file_name:
            dot_wav = ''
        # 保存先のパス
        self.save_path = self.wav_path + file_name + dot_wav
        self.mimic3_plug.get_tts(text, self.save_path)     # 音声作成 and 保存
        self.get_logger().info(f"Save to {self.save_path}")
        return True

    def wav_play(self):
        # .wavファイルまでのパス
        self.get_logger().info(f"Play {self.save_path}")
        # 音声再生
        playsound(self.save_path)
        return True

    def speak(self, srv_req, srv_res):
        _ = self.text_to_wav(srv_req.text, 'mimic3_voice')
        _ = self.wav_play()
        srv_res.result = True
        return srv_res



def main():
    rclpy.init()
    mps = Mimic3PlayServer()
    try:
        rclpy.spin(mps)
    except KeyboardInterrupt:
        pass
    wp.destroy_node()
    rclpy.shutdown()
