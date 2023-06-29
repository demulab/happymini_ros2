from setuptools import setup

package_name = 'detect_pose'

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
    maintainer='demulab',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_left_right = detect_pose.detect_left_right:main',
            'detect_left_right_frontperson = detect_pose.detect_left_right_frontperson:main'
        ],
    },
)
