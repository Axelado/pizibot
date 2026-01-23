from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pizibot_voice'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Axel NIATO',
    maintainer_email='axelniato@gmail.com',
    description='Voice control package for Pizibot robot - enables voice-activated room navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_activator = pizibot_voice.keyboard_activator:main',
            'voice_recorder = pizibot_voice.voice_recorder:main',
            'pose_publish_from_room_number = pizibot_voice.pose_publish_from_room_number:main',
        ],
    },
)
