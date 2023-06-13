from setuptools import setup

package_name = 'rcj_2023_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.xml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusukepad',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carry_my_luggage_2023 = rcj_2023_master.carry_my_luggage_2023:main',
            'find_my_mates_2023 = rcj_2023_master.find_my_mates_2023:main',
            'voice_test = rcj_2023_master.voice_test:main'
        ],
    },
)
