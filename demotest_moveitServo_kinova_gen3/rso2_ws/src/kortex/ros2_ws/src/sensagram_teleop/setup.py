from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensagram_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Kinova Gen3 MoveIt Servo teleop via MediaPipe hand tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_teleop_node = sensagram_teleop.servo_teleop_node:main',
        ],
    },
)
