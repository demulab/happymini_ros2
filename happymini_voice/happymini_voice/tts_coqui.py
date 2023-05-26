import os
from ament_index_python.packages import get_package_share_directory
from TTS.api import TTS
import wave
import pyaudio
import rclpy
from rclpy.node import Node
# Custom msg
from happymini_msgs.srv import TextToSpeech



class TTSCoqui(Node):
    def __init__(self):
        super().__init__('tts_coqui')
        self.create_service(TextToSpeech, 'tts', self.speech)

    def speech(self, srv_req, srv_res):
        tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC")
        tts.tts_to_file(srv_req.text)
        
        # wavファイルのパス
        wav_file = './output.wav'
        
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
    ttsc = TTSCoqui()
    try:
        rclpy.spin(ttsc)
    except KeyboardInterrupt:
        pass
    ttsc.destroy_node()
    rclpy.shutdown()
