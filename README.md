# PIZIBOT

**PIZIBOT** is a ROS 2 Jazzy package for a two-wheeled mobile robot designed for navigation and computer vision.

## Package Overview

| Package | Description | Status |
|---------|-------------|--------|
| `pizibot_description` | Robot URDF and model definitions | ✅ Jazzy |
| `pizibot_hardware` | Real robot launch and configuration | ✅ Jazzy |
| `pizibot_hw_interface` | Hardware interface plugin (ESP32 via serial) | ✅ Jazzy |
| `pizibot_gz` | Gazebo Harmonic simulation | ✅ Jazzy |
| `pizibot_teleop` | Keyboard and joystick teleoperation | ✅ Jazzy |
| `pizibot_navigation` | SLAM and Nav2 navigation stack | ✅ Jazzy |
| `pizibot_voice` | Voice control for room navigation | ✅ Jazzy |
| `pizibot_vision` | Computer vision and camera publisher | ✅ Jazzy |
| `pizibot_remote_ui` | Smartphone PWA — joystick, camera, telemetry | ✅ Jazzy |

## Getting Started

```bash
# Clone and build
cd ~/pizi_ws/src
git clone <this-repo>
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch Files

### Real Robot

```bash
ros2 launch pizibot_hardware launch_real_robot.launch.py
```

Starts: RPLidar · robot_state_publisher · controller_manager · diff_drive_controller · joint_state_broadcaster · twist_mux · ESP32-CAM stream

With web interface:

```bash
ros2 launch pizibot_hardware launch_real_robot.launch.py enable_remote_ui:=true
```

### Simulation (Gazebo Harmonic)

```bash
ros2 launch pizibot_gz launch_sim.launch.py
```

With web interface:

```bash
ros2 launch pizibot_gz launch_sim.launch.py enable_remote_ui:=true
```

### Web Interface (standalone)

```bash
ros2 launch pizibot_remote_ui remote_ui.launch.py
```

Open `http://<robot-ip>:3000` from any device on the same LAN.

### Mapping (SLAM)

```bash
# Start robot first (real or simulation), then:
ros2 launch pizibot_navigation full_mapping.launch.py
```

### Localization and Navigation

```bash
ros2 launch pizibot_navigation full_localization.launch.py map:=/path/to/map.yaml
```

### Camera Publisher

```bash
ros2 launch pizibot_vision camera.launch.py
```

### Dataset Collection (simulation)

```bash
ros2 launch pizibot_gz dataset_save.launch.py
```

## Main Features

- **Motor Control**: Differential drive via `ros2_control` + ESP32 hardware interface
- **Sensor Integration**: RPLidar A1, ESP32-CAM, IMU, battery monitoring
- **SLAM**: Online mapping with SLAM Toolbox
- **Navigation**: Autonomous movement and path planning with Nav2
- **Computer Vision**: Camera image capture, publishing, and semantic segmentation in simulation
- **Voice Control**: Voice-activated room navigation
- **Remote Web UI**: PWA accessible from any smartphone — virtual joystick, live camera, telemetry

## twist_mux Priority Table

All velocity sources publish `TwistStamped` (`use_stamped: true`).

| Source | Topic | Priority |
|--------|-------|----------|
| Keyboard | `cmd_vel_key` | 22 |
| Joystick | `cmd_vel_joy` | 21 |
| Web UI | `cmd_vel_web` | 20 |
| Navigation | `cmd_vel` | 19 |
| Visual tracking | `cmd_vel_tracker` | 18 |

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache-2.0
