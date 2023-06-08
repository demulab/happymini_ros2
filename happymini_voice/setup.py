import os
from glob import glob
from setuptools import setup

package_name = 'happymini_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'sound_effect'),
            glob('config/sound_effect/*')),
        (os.path.join('share', package_name, 'config', 'wav_data', 'cml2023'),
            glob('config/wav_data/cml2023/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='demulab',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_speech = happymini_voice.text_to_speech:main',
            'speech_to_text = happymini_voice.speech_to_text:main',
            'tts_coqui = happymini_voice.tts_coqui:main',
            'wav_play = happymini_voice.wav_play:main'
        ],
    },
)
