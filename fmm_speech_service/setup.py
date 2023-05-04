from setuptools import setup

package_name = 'fmm_speech_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Masaki Ito',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='ROS2 package for speech recognition and speech synthesis and speech make_audio_file',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fmm_client = fmm_speech_service.fmm_client:main',
            'fmm_speech_service = fmm_speech_service.fmm_speech_service:main'
        ],
    },
)
