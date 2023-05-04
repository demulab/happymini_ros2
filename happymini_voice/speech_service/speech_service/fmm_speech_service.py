import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand
import playsound
import wave
import pyaudio
import whisper
import sys
import pyttsx3


model = whisper.load_model("medium")
sys.path.append("/home/yuto/hapyymini_voice/src/happymini_ros2/happymini_voice/whisper/src")

def play_sound():
    sound_file = "/home/yuto/hapyymini_voice/src/happymini_ros2/happymini_voice/mp3/警告音1.mp3"
    playsound.playsound(sound_file, True)

#[*] SpeechServiceクラスをノードとして使うために，Nodeクラスを継承します．
class fmm_SpeechService(rclpy.node.Node):
    def __init__(self):
        #[*] ノードの名前を'speech_service'として登録します．
        super().__init__('fmm_speech_service')

        # ３人の名前を格納する辞書
        self.names = {"male": None, "female": None, "other": None}

        #[*] ノードが起動したことをloggerを用いて表示します．
        self.get_logger().info('音声サーバーを起動しました')

        #[*] yesかnoかを判別するシステムを開始するコマンドを受け取るためのServiceを作成します．
        #[*] リクエストが来たら，command_callback関数を呼び出します．
        self.service = self.create_service(
            StringCommand, '/fmm_speech_service/wake_up', self.command_callback)

        self.names = ["Robert", "Leonardo", "Mary"]        

    def command_callback(self, request, response):
        #[*] サービスを開始したことを音声合成を行い，発話します．
        self.synthesis("Excuse me. May I ask you a few questions ? Please answer yes or no.")
        self.synthesis("Are you a male? Please answer with yes or no")
        self.make_audio_file()

        #[*] 認識した文章が受け取れるまで，音声認識を行います．
        text = None

        while text is None:
            text = self.recognition()

        while True:
        #男性か、女性か？ >>> 男性の場合
            if "yes" in text.lower():
                self.synthesis("OK, Thanks.")
                response.answer_gender = "male"
                #[*] 認識した文章がyesの場合

                #男性の場合
                if response.answer_gender == "male":
                    self.synthesis("Are you Robert ? Please answer yes or no.")
                    self.make_audio_file()
                    text = ""
                    while text == "":
                        text = self.recognition()
 
                    while True:
                #男性の場合で、Robertだったら 
                        if "yes" in text.lower():
                            self.synthesis("OK, Thank you, Robert.")
                            response.answer_name = "Robert"
                            break
                        elif "no" in text.lower():
                            self.synthesis("OK, Thank you, Leonardo")
                            response.answer_name = "Leonardo"
                            break
                    break

                else:
                    pass
                break

                break
            
            #男性か、女性か？ >>> 女性の場合
            elif "no" in text.lower():
                self.synthesis("OK, Thanks.")
                response.answer_gender = "female"
                #女性の場合
                if response.answer_gender == "female":
                    self.synthesis("Are you Elizabeth ? Please answer yes or no.")
                    self.make_audio_file()
                    
                    text = ""
                    while text == "":
                        text = self.recognition()
                    
                    while True:
                #女性の場合で、Elizabethだったら  
                        if "yes" in text.lower():
                            self.synthesis("OK, Thank you,Elizabeth.")
                            response.answer_name = "Elizabeth"
                            break
                        
                        else:
                            pass
                            break
                    break

                else:
                    pass
                    break
    
                break

        #[*] 聞き取れなかった場合
            else:
        #もう一度最初から認識するにょ
                self.synthesis("Sorry, I didn't understand. Please answer yes or no.")
                
                self.make_audio_file()
                text = ""

                while text == "":
                    text = self.recognition()


                while True:
        #男性か、女性か？ >>> 男性の場合
                    if "yes" in text.lower():
                        self.synthesis("OK, Thanks.")
                        response.answer_gender = "male"
 
                        #男性の場合
                        if response.answer_gender == "male":
                            self.synthesis("Are you Robert ? Please answer yes or no.")
                            self.make_audio_file()
                    
                            text = ""
                            while text == "":
                                text = self.recognition()
                            #男性の場合で、Robertだったら、
                            while True:
                                if "yes" in text.lower():
                                    self.synthesis("OK, Thank you, Robert.")
                                    response.answer_name = "Robert"
                                    break

                            #男性の場合で、Leonardoだったら、
                                elif "no" in text.lower():
                                    self.synthesis("OK, Thank you, Leonardo")
                                    response.answer_name = "Leonardo"
                                    break

                                else:
                                    pass
                                    break

                            break
                    
                        break
                
                    #男性か、女性か？ >>> 女性の場合
                    elif "no" in text.lower():
                        self.synthesis("OK, Thanks.")
                        response.answer_gender = "female"
                    #女性の場合 
                        if response.answer_gender == "female":
                            self.synthesis("Are you Elizabeth ? Please answer yes or no.")
                            self.make_audio_file()
                    
                            text = ""
                            while text == "":
                                text = self.recognition()
                    
                            while True:
                        #女性の場合で、Elizabethだったら
                                if "yes" in text.lower():
                                    self.synthesis("OK, Thank you,Elizabeth.")
                                    response.answer_name = "Elizabeth"
                                    break
                        
                                else:
                                    pass
                                    break
                            break
                        break

                    else:     
                        self.synthesis("Sorry, I didn't understand. Please answer yes or no.")
                        text = self.recognition()
        

        #[*] 認識結果をServiceのレンスポンスとして返します．
        response.answer_gender = text
        response.answer_name = text
        return response
    
    def recognition(self):     
    
        text = ''
        # SpeechService = self.create_service(SpeechService, self.speech_service, self.command_callback)
        # SpeechService.make_audio_file()
        # text = self.transcription()
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
