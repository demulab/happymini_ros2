import rclpy
from rclpy.node import Node
from io import BytesIO
import numpy as np
import soundfile as sf
import speech_recognition as sr
import whisper
import wave

from happymini_msgs.srv import SpeechToText


class SpeechToTextServer(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.create_service(SpeechToText, 'stt', self.listen)
        self.get_logger().info("ready ok")

    # whisperでマイクから文字起こし
    def transcription(self, model,recognizer):
        with sr.Microphone(sample_rate=16_000) as source:
            print("なにか話してください")
            audio = recognizer.listen(source)
    
        print("音声処理中 ...")
        wav_bytes = audio.get_wav_data()
        wav_stream = BytesIO(wav_bytes)
        audio_array, sampling_rate = sf.read(wav_stream)
        audio_fp32 = audio_array.astype(np.float32)
    
        res = model.transcribe(audio_fp32, fp16=False,language="en")
        print(res["text"])
        return res["text"]

    def listen(self, srv_req, srv_res):
        #self.get_logger().info(f"req: {srv_req.tex}")
        model = whisper.load_model("medium",device="cpu")
        recognizer = sr.Recognizer()
        result = self.transcription(model, recognizer)
        srv_res.result = result
        return srv_res

def main():
    print('start')
    rclpy.init()
    stts = SpeechToTextServer()
    try:
        rclpy.spin(stts)
        #stts.listen()
    except KeyboardInterrupt:
        pass
    stts.destroy_node()
    rclpy.shutdown()
    print('end')
