# pizibot_voice

Voice control package for the Pizibot robot that enables voice-activated room navigation in ROS 2 Jazzy.

## Overview

**pizibot_voice** is a ROS 2 package that provides voice command recognition and processing capabilities for the Pizibot mobile robot. The system allows users to control the robot's navigation by speaking voice commands in French, such as "va à la salle 1" (go to room 1) or "va en salle 2" (go to room 2).

### Current Features

- 🎤 **Voice Recording**: Records audio from the system microphone with adjustable duration
- 🗣️ **Speech Recognition**: Uses Google Speech Recognition API to transcribe audio to text
- 🇫🇷 **French Language Support**: Optimized for French language commands
- ⌨️ **Keyboard Activation**: Space bar activation for voice recording control
- 🗺️ **Room Navigation**: Publishes goal poses based on recognized room numbers
- 📍 **JSON-based Room Mapping**: Easy configuration of room coordinates via JSON file

## Architecture

The voice control system consists of three main ROS 2 nodes that work together:

```
keyboard_activator (publishes on 'start_talking')
         ↓
    voice_recorder (publishes on 'room_number')
         ↓
pose_publish_from_room_number (publishes on 'goal_pose')
         ↓
    Navigation Stack
```

## Nodes

### 1. keyboard_activator
**Executable**: `keyboard_activator`

Listens for keyboard input (space bar) and publishes activation signals to start/stop voice recording.

**Published Topics**:
- `start_talking` (std_msgs/String): Publishes "1" when space bar is pressed, "0" when released

**Usage**:
```bash
ros2 run pizibot_voice keyboard_activator
```

---

### 2. voice_recorder
**Executable**: `voice_recorder`

Records audio when triggered, performs speech recognition, and extracts room numbers from voice commands.

**Subscribed Topics**:
- `start_talking` (std_msgs/String): Listens for activation signal

**Published Topics**:
- `room_number` (std_msgs/Int16): Publishes the extracted room number (1-99)

**Configuration**:
- Recording duration: 4 seconds (configurable in code)
- Sampling rate: 44100 Hz
- Channels: 2 (stereo)
- Language: French (fr-FR)

**Voice Command Patterns**:
The system recognizes French commands matching these patterns:
- "va à la salle X" (go to room X)
- "va en salle X" (go to room X)
- "va à la salle X" (alternative pattern)
- "va en la salle X" (alternative pattern)

Where X is a number (1-99).

**Usage**:
```bash
ros2 run pizibot_voice voice_recorder
```

---

### 3. pose_publish_from_room_number
**Executable**: `pose_publish_from_room_number`

Converts room numbers to navigation goal poses using a JSON mapping file.

**Subscribed Topics**:
- `room_number` (std_msgs/Int16): Listens for room numbers

**Published Topics**:
- `goal_pose` (geometry_msgs/PoseStamped): Publishes the goal pose in the map frame

**Configuration File**:
- Path: `share/pizibot_voice/data/world_test2_rooms_data.json`
- Format: JSON with room numbers as keys and x, y coordinates as values

**Example JSON Structure**:
```json
{
  "1": {"x": 1.0, "y": 2.5},
  "2": {"x": 3.0, "y": 1.5},
  "3": {"x": 0.5, "y": 4.0}
}
```

**Usage**:
```bash
ros2 run pizibot_voice pose_publish_from_room_number
```

## Installation

### Dependencies

This package requires both system-level dependencies (available via apt) and Python packages (must be installed via pip).

#### System Dependencies

Install keyboard input support:
```bash
sudo apt update
sudo apt install python3-pynput
```

#### Python Dependencies (via pip)

The following Python packages **must** be installed manually as they are not available in Ubuntu repositories:

- `sounddevice` - Audio recording from microphone
- `wavio` - WAV file writing
- `SpeechRecognition` - Google Speech Recognition API wrapper

**ROS 2 Dependencies** (automatically available):
- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - ROS 2 geometry message types
- `std_msgs` - ROS 2 standard message types
- `ament_index_python` - ROS 2 package indexing

---

### Installation Methods

You can install the Python dependencies either **with a virtual environment** (recommended for development) or **without** (simpler for production).

#### Method 1: Installation with Virtual Environment (Recommended)

This method isolates Python dependencies in a virtual environment, preventing conflicts with system packages.

**Step 1: Create and activate virtual environment**
```bash
cd ~/ROS/pizi_ws
python3 -m venv .venv
source .venv/bin/activate
```

**Step 2: Install Python dependencies in venv**
```bash
pip install sounddevice wavio SpeechRecognition
```

**Step 3: Install system dependencies**
```bash
sudo apt install python3-pynput
```

**Step 4: Build the package**
```bash
cd ~/ROS/pizi_ws
colcon build --packages-select pizibot_voice
```

**Step 5: Create setup script for ROS 2 with venv**

Create a file `~/ROS/pizi_ws/setup_with_venv.bash`:
```bash
#!/bin/bash
# Setup script for ROS 2 workspace with Python virtual environment
# Usage: source setup_with_venv.bash

# Activate the Python virtual environment
source ~/ROS/pizi_ws/.venv/bin/activate

# Source the ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source the workspace
source ~/ROS/pizi_ws/install/setup.bash

# Add venv site-packages to PYTHONPATH so ROS 2 nodes can find them
VENV_SITE_PACKAGES="$HOME/ROS/pizi_ws/.venv/lib/python3.12/site-packages"
export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"

echo "✅ ROS 2 workspace with venv activated"
```

Make it executable:
```bash
chmod +x ~/ROS/pizi_ws/setup_with_venv.bash
```

**Step 6: Use the workspace**

In **every new terminal**, source the setup script:
```bash
source ~/ROS/pizi_ws/setup_with_venv.bash
ros2 launch pizibot_voice voice_room_navigation.launch.py
```

**Optional: Add alias to ~/.bashrc**
```bash
echo "alias ws_setup='source ~/ROS/pizi_ws/setup_with_venv.bash'" >> ~/.bashrc
source ~/.bashrc
```

Then simply type `ws_setup` in each terminal.

---

#### Method 2: Installation without Virtual Environment (Simpler)

This method installs Python packages in user space without using a virtual environment.

**Step 1: Install system dependencies**
```bash
sudo apt update
sudo apt install python3-pynput
```

**Step 2: Install Python dependencies in user space**
```bash
pip3 install --user sounddevice wavio SpeechRecognition
```

**Step 3: Build the package**
```bash
cd ~/ROS/pizi_ws
colcon build --packages-select pizibot_voice
```

**Step 4: Source the workspace**
```bash
source ~/ROS/pizi_ws/install/setup.bash
```

**Step 5: Run the package**
```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py
```

---

### Verifying Installation

To verify that all dependencies are correctly installed:

**Test Python imports:**
```bash
python3 -c "import sounddevice; import wavio; import speech_recognition; print('✅ All Python packages found')"
```

**Test ROS 2 package:**
```bash
ros2 pkg list | grep pizibot_voice
```

**Test node executables:**
```bash
ros2 run pizibot_voice keyboard_activator --help
```

## Usage

### Quick Start - Voice Room Navigation

Launch the complete voice navigation stack:

```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py
```

**With simulation time enabled**:
```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py use_sim_time:=true
```

**With real robot (use_sim_time=false)**:
```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py use_sim_time:=false
```

### Manual Node Startup

If you want to run individual nodes:

```bash
# Terminal 1: Start the keyboard activator
ros2 run pizibot_voice keyboard_activator

# Terminal 2: Start the voice recorder
ros2 run pizibot_voice voice_recorder

# Terminal 3: Start the pose publisher
ros2 run pizibot_voice pose_publish_from_room_number
```

### Testing with ROS 2 CLI

**Publish a room number directly**:
```bash
ros2 topic pub /room_number std_msgs/msg/Int16 "data: 1" --once
```

**Monitor goal poses**:
```bash
ros2 topic echo /goal_pose
```

**Monitor voice activation**:
```bash
ros2 topic echo /start_talking
```

## Configuration

### Room Mapping File

The room coordinates are defined in `data/world_test2_rooms_data.json`. Edit this file to configure rooms for your environment:

```json
{
  "1": {"x": 1.0, "y": 2.5},
  "2": {"x": 3.0, "y": 1.5},
  "3": {"x": 0.5, "y": 4.0},
  "4": {"x": 5.5, "y": 3.0}
}
```

### Launch File Arguments

The launch file supports the following parameters:

- `use_sim_time` (bool, default: true): Use simulation time instead of system time
- `room_data_path` (string, default: package share data/world_test2_rooms_data.json): Path to the JSON mapping file

Example:
```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py use_sim_time:=false
```

## Troubleshooting

### Issue: No sound input detected

**Solution**:
- Check that your microphone is properly connected and enabled
- Verify audio permissions: `sudo usermod -a -G audio $USER`
- Test microphone: `arecord -D default -f cd -t wav | aplay`

### Issue: Google Speech Recognition fails

**Solution**:
- Ensure internet connection is available
- Check network connectivity: `ping google.com`
- The API requires an active internet connection to work
- See `voice_recorder.py` logs for detailed error messages

### Issue: Voice commands not recognized

**Solution**:
- Ensure audio is clear and without excessive background noise
- Speak slowly and clearly
- Check that commands match the expected French patterns:
  - "va à la salle 1"
  - "va en salle 2"
- View logs to see what was actually recognized: `ros2 run pizibot_voice voice_recorder --log-level DEBUG`

### Issue: Room number not found

**Solution**:
- Verify the room number exists in `world_test2_rooms_data.json`
- Check that the JSON file is properly formatted
- Ensure the file is in the correct location in the installed package

### Issue: Permission denied for keyboard input

**Solution**:
- Run with appropriate permissions if needed
- The keyboard listener might require elevated privileges on some systems
- Alternative: Use ROS 2 topic publishing directly instead of keyboard activation

## Package Contents

```
pizibot_voice/
├── pizibot_voice/              # Main package source code
│   ├── __init__.py
│   ├── keyboard_activator.py   # Keyboard activation node
│   ├── voice_recorder.py       # Voice recording and recognition node
│   └── pose_publish_from_room_number.py  # Pose publisher node
├── launch/
│   └── voice_room_navigation.launch.py   # Main launch file
├── data/
│   └── world_test2_rooms_data.json       # Room coordinates mapping
├── resource/
│   └── pizibot_voice           # Package resource marker
├── package.xml                 # ROS 2 package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Setup configuration
└── README.md                   # This file
```

## Message Types

### start_talking (std_msgs/String)
```
data: "1"  # Space bar pressed (start recording)
data: "0"  # Space bar released (stop recording)
```

### room_number (std_msgs/Int16)
```
data: 1  # Room 1
data: 2  # Room 2
# ... etc
```

### goal_pose (geometry_msgs/PoseStamped)
```
header:
  stamp: [timestamp]
  frame_id: "map"
pose:
  position:
    x: 1.0
    y: 2.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## Parameters

All nodes support the standard ROS 2 parameter `use_sim_time`:

```bash
ros2 launch pizibot_voice voice_room_navigation.launch.py use_sim_time:=true
```

## Limitations

- **Language**: Currently optimized for French commands only
- **Room numbers**: Limited to integers (1-99)
- **Internet**: Requires active internet connection for Google Speech Recognition
- **Audio**: Requires functional microphone and audio input device
- **Command format**: Only recognizes specific French command patterns
- **Keyboard**: Keyboard listener requires appropriate system permissions

## Future Improvements

- [ ] Support for additional languages
- [ ] Offline speech recognition (e.g., using Vosk)
- [ ] Enhanced voice command parsing with NLP
- [ ] Noise filtering and audio preprocessing
- [ ] Command confirmation feedback with text-to-speech
- [ ] Voice activity detection (VAD)
- [ ] Confidence scoring for recognized commands
- [ ] Machine learning-based command classification

## Author

**Axel NIATO** - axelniato@gmail.com

## License

Apache License 2.0

## Acknowledgments

- Google Speech Recognition API for speech-to-text conversion
- ROS 2 community for the excellent robotics framework
- Pizibot project team

## Support

For issues, questions, or feature requests, please refer to the main [PIZIBOT README](../README.md) or contact the package maintainer.

---

**Last Updated**: January 23, 2026  
**ROS 2 Distribution**: Jazzy  
**Python Version**: Python 3.10+
