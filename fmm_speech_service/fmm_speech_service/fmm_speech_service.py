import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand
import playsound
import wave
import pyaudio
import whisper
import sys
import pyttsx3
import random

model = whisper.load_model("medium")
sys.path.append("/home/yuto/happymini_voice_offline/src/happymini_ros2/happymini_voice/whisper/src")

def play_sound():
    sound_file = "/home/masaki/tututest_ws/src/happymini_ros2/happymini_voice/mp3/警告音1.mp3"
    playsound.playsound(sound_file, True)

#[*] SpeechServiceクラスをノードとして使うために，Nodeクラスを継承します．
class fmm_SpeechService(rclpy.node.Node):
    def __init__(self):
        #[*] ノードの名前を'speech_service'として登録します．
        super().__init__('fmm_speech_service')

        self.answer_gender = ''
        self.answer_name = ''

        #[*] ノードが起動したことをloggerを用いて表示します．
        self.get_logger().info('音声サーバーを起動しました')

        #[*] yesかnoかを判別するシステムを開始するコマンドを受け取るためのServiceを作成します．
        #[*] リクエストが来たら，command_callback関数を呼び出します．
        self.service = self.create_service(
            StringCommand, '/fmm_speech_service/wake_up', self.name_and_gender_callback)

        # 3人の名前と性別を格納するリスト
        self.people = [("male", "Robert"), ("male", "Leonardo"), ("female", "Elizabeth")]

    def name_and_gender_callback(self, request, response):

        self.synthesis("Excuse me, I have a few questions. Please answer with yes or no.")

        for gender, name in self.people:
            self.synthesis(f"Are you {name}? Please answer with yes or no")
            self.make_audio_file()
            
            # 認識した文章が受け取れるまで，音声認識を行います．
            text = None
            while text is None:
                text = self.recognition()

            while True:
                if "yes" in text.lower():
                    self.synthesis(f"OK, Thank you.")
                    response.answer_gender = gender
                    response.answer_name = name
                    response.answer_people = [name, gender]
                    response.answer = "OK"
                    break

                elif "no" in text.lower():                  
                    break

                else:
                    self.synthesis("Sorry, I didn't understand. Please answer yes or no.")
                    self.make_audio_file()
                    text = ""
                    while text == "":
                        text = self.recognition()

            if response.answer == "OK":
                break

        return response
   
    def recognition(self):     
    
        text = ''
        audio = whisper.load_audio("output.wav")
        audio = whisper.pad_or_trim(audio)
        dictionary = ["YES","Yes.","Yes!","yes","no","イエス","ノー","Yes","No"]
        yesdic = ["YES","Yes!","Yes.","Yes","yes","イエス",]
        nodic = ["No","no","ノー"]
        
        # audioから取得した音声データをメル周波数スペクトログラムに変換する
        mel = whisper.log_mel_spectrogram(audio).to(model.device)
        
        # Melスペクトログラムで表される入力音声信号の言語を予測するために言語検出モデルを使用しています。
        # model.detect_language(mel) は、Melスペクトログラムであるmelを入力として受け取り、言語検出モデルを適用する関数です。
        _,probs = model.detect_language(mel)
        # max(probs, key=probs.get) は、probs辞書の中で最も確率が高い言語を見つけるために使用されます。
        print(f"Detected language:{max(probs,key=probs.get)}")

        options = whisper.DecodingOptions(fp16 = False)
        result = whisper.decode(model, mel, options)
        text = result.text
        word = text.split()
        matched_words = []
        print(text)

        # [*] 認識した結果をloggerで表示します．
        self.get_logger().info(f'認識したテキストは "{text}" です')

        return text
    
    def make_audio_file(self):
        play_sound()
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        RECORD_SECONDS = 5
        WAVE_OUTPUT_FILENAME = "output.wav"

        p = pyaudio.PyAudio()

        stream = p.open(format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                input = True,
                    #output = True,make_audio_file(self):
                frames_per_buffer = CHUNK)

        print("recording")
        frames = []
        # 1回あたりCHUNK（1024）個のフレームを読み込んで、合計で指定された時間(RECORD_SECONDS)だけ音声を録音し、framesに音声データを追加しています。
        for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
            d = stream.read(CHUNK)
            frames.append(d)

        print("done recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
    
    def synthesis(self, text):
        # 音声合成を行うことをloggerで表示します．
        self.get_logger().info(f'音声合成を実行します')
        self.get_logger().info(f'発話内容は "{text}"')

        # 音声合成エンジンを初期化します．
        engine = pyttsx3.init()

        # 言語を設定します．
        self.lang = "en-US"
        engine.setProperty('voice', self.lang)
        # rateは、1分あたりの単語数で表した発話速度。基本は、200です。
        rate = engine.getProperty("rate")
        engine.setProperty("rate",160)

        # ボリュームは、0.0~1.0の間で設定します。
        volume = engine.getProperty('volume')
        engine.setProperty('volume',1.0)

        # テキストを音声に合成します．
        engine.say(text)

        engine.runAndWait()

def main():
    #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    rclpy.init()

    #[*] yesかnoかを判別するシステムのサービスを初期化します．
    fmm_speech_service = fmm_SpeechService()

    try:
    #[*] 音声認識のノードを実行します．
        rclpy.spin(fmm_speech_service)
    except KeyboardInterrupt:
        pass

    #[*] rclpyを通したrosのコミュニケーションを終了します．
    rclpy.shutdown()
