from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pizibot_voice'
launch_files = glob(os.path.join('launch', '*.launch.py'))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/rooms_data.json']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools', 'pynput', 'sounddevice', 'wavio', 'SpeechRecognition', 'gtts'],
    zip_safe=True,
    maintainer='axel',
    maintainer_email='axelniato@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_activator = pizibot_voice.keyboard_activator:main",
            "voice_recorder = pizibot_voice.voice_recorder:main",
            "pose_publish_from_room_number = pizibot_voice.pose_publish_from_room_number:main"
        ],
    },
)
