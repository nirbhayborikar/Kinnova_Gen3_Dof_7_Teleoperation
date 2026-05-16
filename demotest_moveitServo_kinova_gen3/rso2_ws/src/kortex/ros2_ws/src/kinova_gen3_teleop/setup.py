from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kinova_gen3_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'teleop_node = kinova_gen3_teleop.teleop_node:main',
            'arm_controller = kinova_gen3_teleop.arm_controller_node:main',
        ],
    },
)
