import os
from ament_index_python.packages import get_package_share_directory

from ovos_tts_plugin_mimic3 import Mimic3TTSPlugin

import wave
import pyaudio
import rclpy
from rclpy.node import Node
# Custom msg
from happymini_msgs.srv import TextToSpeech



class TTSmimic3(Node):
    def __init__(self):
        super().__init__('tts_mimic3')
        self.create_service(TextToSpeech, 'tts', self.speech)

    def speech(self, srv_req, srv_res):
        mimic = Mimic3TTSPlugin(lang="en")
        mimic.get_tts(srv_req.text, "mimic3.wav", voice="en_US/cmu-arctic_low", speaker="eey")
        
        # wavファイルのパス
        wav_file = './mimic3.wav'
        
        # waveファイルを開く
        wave_file = wave.open(wav_file, 'rb')
        
        # PyAudioを初期化
        audio = pyaudio.PyAudio()
        
        # ストリームを開く
        stream = audio.open(
            format=audio.get_format_from_width(wave_file.getsampwidth()),
            channels=wave_file.getnchannels(),
            rate=wave_file.getframerate(),
            output=True
        )
        
        # データを読み込んで再生
        data = wave_file.readframes(1024)
        while data:
            stream.write(data)
            data = wave_file.readframes(1024)
        
        # ストリームとPyAudioを閉じる
        stream.stop_stream()
        stream.close()
        audio.terminate()

        srv_res.result = True
        return srv_res

def main():
    rclpy.init()
    ttsm = TTSmimic3()
    try:
        rclpy.spin(ttsm)
    except KeyboardInterrupt:
        pass
    ttsc.destroy_node()
    rclpy.shutdown()
