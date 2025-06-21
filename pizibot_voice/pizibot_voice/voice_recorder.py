"""
voice_recorder.py

This ROS 2 node listens for a trigger on the 'start_talking' topic to record the user's voice,
recognizes the spoken command, extracts the room number, and publishes it on the 'room_number' topic.

Workflow:
    1. Waits for a message "1" on the 'start_talking' topic to start recording.
    2. Records audio from the microphone for a fixed duration and saves it as a WAV file.
    3. Uses Google Speech Recognition to transcribe the audio to text.
    4. Extracts the room number from commands like "va à la salle X" or "va en salle Y" (French).
    5. Publishes the extracted room number as an Int16 message on the 'room_number' topic.

Usage:
    - Run as part of the voice_room_navigation.launch.py launch file or standalone.
    - Press and hold the space bar (using keyboard_activator) to trigger voice recording.

Dependencies:
    - sounddevice
    - wavio
    - speech_recognition
    - pynput (for keyboard_activator)
    - Google Speech Recognition API (internet required)

Author: Axel NIATO
Date: 20/06/2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory
import os

import sounddevice as sd
import wavio
import speech_recognition as sr
import re

package_name = "pizibot_voice"

class VoiceRecorder(Node):
    def __init__(self):
        super().__init__('voice_recorder')
        self.publisher_ = self.create_publisher(Int16, 'room_number', 10)
        self.subscription = self.create_subscription(
            String,
            'start_talking',  
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        if msg.data == "1":
            file_path = self.recorder()
            text = self.recognizer(file_path)
            if text != None:
                room_number = self.commande_interpretor(text)
                if room_number != None:
                    self.publish_room_number(room_number)
                
            
    def recorder(self):
        # Recording parameters
        duration = 4  # Recording duration in seconds
        sample_rate = 44100  # Sampling rate in Hz

        self.get_logger().info("Recording in progress...")
        audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=2, dtype='int16')
        sd.wait()  # Wait for the recording to finish
        self.get_logger().info("Recording finished.")

        # Save the audio to a file
        directory_path = os.path.join(get_package_share_directory(package_name), 'records')
        os.makedirs(directory_path, exist_ok=True)
        
        filename = "enregistrement.wav"
        file_path = os.path.join(directory_path, filename)
        
        wavio.write(file_path, audio_data, sample_rate, sampwidth=2)
        self.get_logger().info(f"The recording has been saved as '{file_path}'.")
        
        return file_path
        
    def recognizer(self, file_path):
        # Initialize the speech recognizer
        r = sr.Recognizer()

        audio_file = file_path
        try:
            # Use the audio file instead of the microphone
            with sr.AudioFile(audio_file) as source:
                self.get_logger().info("Reading audio file...")
                audio_data = r.record(source)  # Read the entire audio file
                self.get_logger().info("End reading!")

            # Speech recognition
            try:
                result = r.recognize_google(audio_data, language="fr-FR")
                # For English speech recognition
                # result = r.recognize_google(audio_data, language="en-EN")
                self.get_logger().info(f"You said: {result}")
            except sr.UnknownValueError:
                self.get_logger().info("Google Speech Recognition could not understand the audio.")
                result = None
            except sr.RequestError as e:
                self.get_logger().info(f"Request error with Google Speech Recognition; {e}")
                result = None
        except FileNotFoundError:
            self.get_logger().info(f"The audio file {audio_file} was not found.")
            result = None
        except Exception as e:
            self.get_logger().info(f"An error occurred while reading the audio file: {e}")
            result = None
            
        if os.path.exists(audio_file):
            try:
                os.remove(audio_file)
                self.get_logger().info(f"The audio file {audio_file} has been deleted.")
            except Exception as e:
                self.get_logger().info(f"An error occurred while deleting the audio file: {e}")
        return result
            
    def commande_interpretor(self, commande):
        # Regular expression to identify commands like "va à la salle X" or "va en salle Y"
        pattern = re.compile(r"va\s+(?:à\s+la\s+salle?\s*(\d+)|en\s+(?:la\s+)?salle?\s*(\d+))", re.IGNORECASE)
        match = pattern.match(commande.strip())

        if match:
            # Extract the room number
            room_number = match.group(1) or match.group(2)
            self.get_logger().info(f"Valid command: {commande} -> Room number: {room_number}")
            return int(room_number)
        
        else:
            self.get_logger().info(f"Invalid command: {commande}")
            return None

    def publish_room_number(self, room_number):
        msg = Int16()
        msg.data = int(room_number)
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
