import os
from glob import glob
from setuptools import setup

package_name = 'yolov8_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luiz',
    maintainer_email='luizcarloscosmifilho@gmail.com',
    description='Object Detection with YoloV8',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector = yolov8_ros2.main:main',
            'render_detections = yolov8_ros2.view:main',
        ],
    },
)
