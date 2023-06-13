import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import soundfile as sf
import speech_recognition as sr
import playsound
import whisper
import wave
import math
import rclpy
from rclpy.node import Node
from io import BytesIO
# Custom msg
from happymini_msgs.srv import SpeechToText


class SpeechToTextServer(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.create_service(SpeechToText, 'stt', self.listen)
        self.get_logger().info("Ready ...")
        self.signal_file = os.path.join(
                get_package_share_directory('happymini_voice'),
                'config',
                'signal_sound.mp3')

    # whisperでマイクから文字起こし
    def transcription(self, model, recognizer):
        with sr.Microphone(sample_rate=16_000) as source:
            self.get_logger().info("Please say anything ...")
            playsound.playsound(self.signal_file, True)
            #print(recognizer.energy_threshold)
            recognizer.dynamic_energy_threshold = False
            distance = 0.5
            #recognizer.energy_threshold = recognizer.energy_threshold + 60
            x = recognizer.energy_threshold
            dis = 0.5/distance
            sik = 300/x
            y = math.sqrt((x+60)**2/x**2-1)
            recognizer.energy_threshold = math.sqrt(x**2+(x*dis*sik*y)**2)
            #recognizer.energy_threshold = recognizer.energy_threshold * 1.8 * (0.5/distance)
            #print(recognizer.energy_threshold)
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
            except sr.exceptions.WaitTimeoutError:
                pass
    
        #print("音声処理中 ...")
        self.get_logger().info("Loading ...")
        wav_bytes = audio.get_wav_data()
        wav_stream = BytesIO(wav_bytes)
        audio_array, sampling_rate = sf.read(wav_stream)
        audio_fp32 = audio_array.astype(np.float32)
    
        res = model.transcribe(audio_fp32, fp16=False,language="en")
        #print(res["text"])
        return res["text"]

    def yes_no(self, text):
        yes_no = 'yes'
        lower_text = text.lower()
        self.get_logger().info(f"{lower_text}")
        if 'yes' in lower_text:
            yes_no = 'yes'
        elif 'no' in lower_text:
            yes_no = 'no'
        else:
            yes_no = 'NULL'
        return yes_no

    def listen(self, srv_req, srv_res):
        self.get_logger().info(f"Command: {srv_req.cmd}")
        model = whisper.load_model("medium",device="cpu") #medium
        _ = model.half()
        _ = model.cuda()
        for m in model.modules():
            if isinstance(m, whisper.model.LayerNorm):
                m.float()
        recognizer = sr.Recognizer()
        recognizer.dynamic_energy_threshold = True
        result = self.transcription(model, recognizer)
        self.get_logger().info(f"{result}")
        srv_res.result = result
        if srv_req.cmd == 'yes_no':
            srv_res.result = self.yes_no(result)
        return srv_res

def main():
    rclpy.init()
    stts = SpeechToTextServer()
    try:
        rclpy.spin(stts)
        #stts.listen()
    except KeyboardInterrupt:
        pass
    stts.destroy_node()
    rclpy.shutdown()
    #print('end')
