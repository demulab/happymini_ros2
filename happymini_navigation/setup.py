import os
from glob import glob
from setuptools import setup

package_name = 'happymini_navigation'

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
        (os.path.join('share', package_name, 'location'),
            glob('location/*.yaml'))
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
            'set_location = happymini_navigation.set_location:main',
            'navi_location = happymini_navigation.navi_location:main',
            'set_params = happymini_navigation.set_params:main'
        ],
    },
)
