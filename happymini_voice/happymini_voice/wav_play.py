import os
import wave
import yaml
from gtts import gTTS
from playsound import playsound
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
# Custom msgs
from happymini_msgs.srv import WavPlay, WavMake


class WavPlayServer(Node):
    def __init__(self):
        super().__init__('wav_play_node')
        self.create_service(WavPlay, 'wav_play_server', self.wav_play)
        self.get_logger().info("Ready to set /wav_play_server")
        self.create_service(WavMake, 'wav_make_server', self.wav_make)
        self.get_logger().info("Ready to set /wav_make_server")
        self.pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.wav_path = os.path.join(
                self.pkg_path,
                'config',
                'wav_data')

    def text_to_wav(self, file_path, text):
        # 「file_path」に「.wav」がなかったらつける
        dot_wav = '.wav'
        if '.wav' in file_path:
            dot_wav = ''
        # 保存先のパス
        save_path = self.wav_path + file_path + dot_wav
        tts = gTTS(text)     # 音声作成
        tts.save(save_path)  # 保存
        self.get_logger().info(f"Save to {save_path}")
        return True

    def yaml_to_wav(self, package_name, file_name):
        dot_yaml = '.yaml'
        if '.yaml' in file_name:
            dot_yaml = ''
        # YAMLファイルまでのパス
        yaml_path = os.path.join(
                get_package_share_directory(package_name),
                'config',
                file_name + dot_yaml)
        with open(yaml_path) as path:
            yaml_file = yaml.safe_load(path)
        # 1つずつ音声にして保存
        for key, value in yaml_file.items():
            result = self.text_to_wav(key, value)
        return result

    def wav_make(self, srv_req, srv_res):
        # パッケージ名が指定されてたらYAMLファイルから音声作成
        if srv_req.package != 'None':
            srv_res.result = self.yaml_to_wav(srv_req.package, srv_req.file)
            return srv_res
        # 音声作成
        srv_res.result = self.text_to_wav(srv_req.file, srv_req.text)
        return srv_res

    def wav_play(self, srv_req, srv_res):
        # .wavファイルまでのパス
        wav_file = self.wav_path + srv_req.file + '.wav'
        self.get_logger().info(f"Play {wav_file}")
        # 音声再生
        playsound(wav_file)
        srv_res.result = True
        return srv_res


def main():
    rclpy.init()
    wp = WavPlayServer()
    try:
        rclpy.spin(wp)
    except KeyboardInterrupt:
        pass
    wp.destroy_node()
    rclpy.shutdown()
