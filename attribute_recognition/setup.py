from setuptools import setup

package_name = 'attribute_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: ["resource/**"]},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name, "resource/out/deepmar.index", "resource/out/deepmar.data-00000-of-00001"]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qibitech',
    maintainer_email='k.demura@qibitech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "attribute_recog_node = attribute_recognition.attribute_recog_node:main "
        ],
    },
)
