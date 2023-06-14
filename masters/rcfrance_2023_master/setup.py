from setuptools import setup
import os
from glob import glob

package_name = 'rcfrance_2023_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
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
            'carry_my_luggage_france2023 = rcfrance_2023_master.carry_my_luggage_france2023:main',
            'receptionist_france2023 = rcfrance_2023_master.receptionist_france2023:main'
        ],
    },
)
