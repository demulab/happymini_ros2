###################################
# VOICEVOXを起動させてから実行する
###################################
#マイクからwhisperで文字起こし
from io import BytesIO
import numpy as np
import soundfile as sf
import speech_recognition as sr
import whisper
import torch
import requests
#VOICEVOXのサウンド
from playsound import playsound
# import pyaudio


# whisperでマイクから文字起こし
def transcription(model,recognizer):
    # with sr.Microphone(device_index=11) as source:
    with sr.Microphone(sample_rate=16_000) as source:
        print("なにか話してください")
        audio = recognizer.listen(source)

    print("音声処理中 ...")
    wav_bytes = audio.get_wav_data()
    wav_stream = BytesIO(wav_bytes)
    audio_array, sampling_rate = sf.read(wav_stream)
    audio_fp32 = audio_array.astype(np.float32)

    result = model.transcribe(audio_fp32, fp16=False,language="en")
    print(result["text"])
    return result

# VOICEVOXで音声合成
def generate_wav(text, speaker=8, filepath='./audio.wav'):
    host = 'localhost'
    port = 50021
    params = (
        ('text', text),
        ('speaker', speaker),
    )
    response1 = requests.post(
        f'http://{host}:{port}/audio_query',
        params=params
    )
    headers = {'Content-Type': 'application/json',}
    response2 = requests.post(
        f'http://{host}:{port}/synthesis',
        headers=headers,
        params=params,
        data=json.dumps(response1.json())
    )

    wf = wave.open(filepath, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(24000)
    wf.writeframes(response2.content)
    wf.close()

if __name__ == "__main__":
    # wisperの設定
    torch.cuda.is_available = lambda: False
    model = whisper.load_model("base",device="cpu")
    # model = whisper.load_model("small",device="cpu")
    # _ = model.half()
    # _ = model.cuda()
    # exception without following code
    # reason : model.py -> line 31 -> super().forward(x.float()).type(x.dtype)
    # for m in model.modules():
    #     if isinstance(m, whisper.model.LayerNorm):
    #         m.float()
    recognizer = sr.Recognizer()

    while True:
        # whisperマイクから文字起こし
        result = transcription(model,recognizer)
        # 音声合成
        # generate_wav(r.json()["bestResponse"]["utterance"])
        # 音声再生
        # playsound("audio.wav")
