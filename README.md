# PIZIBOT

**PIZIBOT** is a ROS 2 package for a two-wheeled mobile robot designed for navigation and computer vision.

## Project Status

🚀 **ROS 2 Jazzy Migration in Progress**

The project is under active development with ongoing migration to **ROS 2 Jazzy** and **Gazebo Harmonic**.

### Migration Status
- ✅ **Migrated to Jazzy**:
  - `pizibot_description` - Robot URDF and model definitions
  - `pizibot_gz` - Gazebo Harmonic simulation (replaces `pizibot_gazebo`)
  - `pizibot_teleop` - Keyboard and joystick teleoperation
  - `pizibot_navigation` - SLAM and Nav2 navigation stack
  - `pizibot_voice` - Voice control for room navigation
- ⏳ **Pending Migration**:
  - `pizibot_hardware` - Real robot hardware interface and battery management
  - `pizibot_vision` - Computer vision package

Basic functionalities for robot description, control, SLAM, and navigation are now available.  
You will find launch files for both simulation and real robot usage.

## ⚠️ Known Limitations (Not Yet Ready)

### Packages Not Yet Migrated to Jazzy

The following packages are **still based on ROS 2 Humble** and require migration:

#### 1. **pizibot_hardware** - Real Robot Interface
- **Status**: Built for ROS 2 Humble
- **Features developed**:
  - Battery management and monitoring
  - Motor control via hardware interface
- **What's missing**:
  - Jazzy compatibility updates
  - Testing on real hardware with Jazzy
  - Hardware dependencies verification
- **Workaround**: Use only with ROS 2 Humble on real robot hardware

#### 2. **pizibot_vision** - Computer Vision
- **Status**: Planned, not yet implemented
- **What's needed**:
  - Object detection pipeline
  - Camera calibration and configuration
  - ROS 2 Jazzy integration
- **Timeline**: To be implemented

### Current Limitations

- **Real Robot Testing**: Hardware interface packages are not yet Jazzy-compatible
- **Computer Vision**: Not yet implemented
- **Web Interface**: Not yet developed
- **LCD Display Integration**: Not yet implemented

### Recommended Usage

- **🎮 For Simulation Testing**: Use `pizibot_description`, `pizibot_gz`, `pizibot_teleop`, `pizibot_navigation`, and `pizibot_voice` (all Jazzy-ready)
- **🤖 For Real Robot**: Still requires ROS 2 Humble environment until `pizibot_hardware` is migrated
- **🗺️ For Navigation/SLAM**: Use `pizibot_navigation` with Gazebo Harmonic simulation (Jazzy-compatible)
- **🎙️ For Voice Control**: Use `pizibot_voice` for voice-activated room navigation (Jazzy-compatible)

## Main Features

- **General Robot Management**:
  - **Battery Management**: Battery level monitoring and automatic safety actions. **(Implemented)**
  - **LCD Display Integration**: 16x2 LCD for real-time robot status (battery, connection, etc.). **(Not yet implemented)**
- **Motor Control**: Differential drive motor control. **(Implemented)**
- **Sensor Integration**: Support for LIDAR, cameras, and other sensors. **(Implemented)**
- **SLAM**: Online mapping with SLAM Toolbox. **(Implemented)**
- **Navigation**: ROS 2 Navigation (Nav2) for autonomous movement and path planning. **(Implemented)**
- **Computer Vision**: Object detection and tracking with onboard cameras. **(Not yet implemented)**
- **Web Interface**: Remote control and monitoring via a web browser. **(Not yet implemented)**

## Launch Files

- **Simulation (Gazebo Harmonic - Jazzy)**:  
  Launch the robot in Gazebo Harmonic with:

  ```bash
  ros2 launch pizibot_gz launch_sim.launch.py
  ```
  
  > **Note:** `pizibot_gazebo` (Gazebo Classic) has been replaced by `pizibot_gz` (Gazebo Harmonic)

- **Mapping (SLAM) – Simulation or Real Robot**:  

  ```bash
  ros2 launch pizibot_navigation full_mapping.launch.py
  ```

  > **Note:** You must start the simulation or the real robot before running this launch file.

- **Localization & Navigation – Simulation or Real Robot**:  

  ```bash
  ros2 launch pizibot_navigation full_localization.launch.py
  ```

  > **Note:** You must start the simulation or the real robot before running this launch file.

## Getting Started

1. **Clone the repository and build the workspace:**

    ```bash
    cd ~/ROS/pizi_ws/src
    git clone <this-repo>
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source install/setup.bash
    ```

2. **Start the simulation or connect your real robot.**

3. **Use the provided launch files for mapping or localization/navigation as described above.**

## Documentation

- Each launch file contains detailed comments and usage instructions.
- For more details on configuration and usage, see the documentation in each package.

---

**Contributions are welcome!**  
Feel free to open issues or submit pull requests.
