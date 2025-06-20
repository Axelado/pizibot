# PIZIBOT

**PIZIBOT** is a ROS 2 package for a two-wheeled mobile robot designed for navigation and computer vision.

## Project Status

The project is under active development.  
Basic functionalities for robot description, control, SLAM, and navigation are now available.  
You will find launch files for both simulation and real robot usage.

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

- **Simulation**:  
  Launch the robot in Gazebo with:

  ```bash
  ros2 launch pizibot_gazebo launch_sim.launch.py
  ```

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
