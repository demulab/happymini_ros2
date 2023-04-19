from setuptools import setup

package_name = 'grasp_bag'

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
    maintainer='yusukepad',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_data_sensing_node = grasp_bag.scan_data_sensing_node:main',
            'bag_localization_node = grasp_bag.bag_localization_node:main',
            'grasp_bag_server = grasp_bag.grasp_bag_server:main',
            'test_node = grasp_bag.test_node:main'
        ],
    },
)
