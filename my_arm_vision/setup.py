from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_arm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'model'),
         glob(os.path.join('model', '*.pt'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='sherlock',
    maintainer_email='l17876079248@gmail.com',
    description='ROS2 视觉引导抓取节点：YOLOv8 目标检测',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = my_arm_vision.yolo_detector:main',
            'pose_transformer = my_arm_vision.pose_transformer:main',
            'websocket_server = my_arm_vision.websocket_server:main',
        ],
    },
)
