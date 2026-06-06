# Pizibot Hardware

ROS 2 Jazzy launch and configuration package for the real Pizibot robot.

## Description

This package orchestrates all hardware components of the real robot:

- **RPLidar A1** — laser scanner
- **ros2_control** — controller manager with `diff_drive_controller` and `joint_state_broadcaster`
- **twist_mux** — velocity command multiplexer (navigation, joystick, keyboard, web UI, visual tracking)
- **ESP32-CAM** — HTTP video stream → `/camera/image_raw`
- **pizibot_hw_interface** — hardware interface plugin (serial communication with ESP32)

## Status

✅ Fully migrated to **ROS 2 Jazzy**.

## Usage

```bash
ros2 launch pizibot_hardware launch_real_robot.launch.py
```

With the web remote interface:

```bash
ros2 launch pizibot_hardware launch_real_robot.launch.py enable_remote_ui:=true
```

### Verify hardware is up

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

### Send a velocity command

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

## Package Structure

```text
pizibot_hardware/
├── launch/
│   ├── launch_real_robot.launch.py   # Main real robot launch
│   └── rplidar.launch.py             # RPLidar A1 launch
├── param/
│   ├── robot_controllers.yaml        # diff_drive_controller + joint_state_broadcaster
│   └── twist_mux.yaml                # Velocity multiplexer sources and priorities
├── scripts/
│   └── esp32cam_stream_getter.py     # ESP32-CAM HTTP stream → ROS Image
└── rviz/
    └── viz_real_bot.rviz             # RViz preset for real robot
```

## Configuration

### Controller parameters (`param/robot_controllers.yaml`)

| Parameter | Value |
| --------- | ----- |
| `wheel_separation` | 0.29 m |
| `wheel_radius` | 0.04 m |
| `update_rate` | 50 Hz |
| `cmd_vel_timeout` | 0.5 s |
| `enable_odom_tf` | true |

### Twist mux priorities (`param/twist_mux.yaml`)

All sources publish `TwistStamped` (`use_stamped: true`).

| Source | Topic | Priority |
| ------ | ----- | -------- |
| Keyboard | `cmd_vel_key` | 22 |
| Joystick | `cmd_vel_joy` | 21 |
| Web UI | `cmd_vel_web` | 20 |
| Navigation | `cmd_vel` | 19 |
| Visual tracking | `cmd_vel_tracker` | 18 |

Output remapped to `diff_drive_controller/cmd_vel`.

### Launch arguments

| Argument             | Default  | Description                                                       |
|----------------------|----------|-------------------------------------------------------------------|
| `enable_remote_ui`   | `false`  | Launch rosbridge + web_video_server + HTTP server for the web UI  |

### Serial port

The ESP32 serial port is configured in `pizibot_description/urdf/control.xacro`:

```xml
<param name="serial_port">/dev/ttyESP32</param>
```

Create a udev rule to get a stable device name:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
SYMLINK+="ttyESP32", MODE="0666"
```

Adapt `idVendor`/`idProduct` using `lsusb`. Then reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## ROS Topics

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `/diff_drive_controller/cmd_vel` | `TwistStamped` | Velocity command input (twist_mux output) |
| `/diff_drive_controller/odom` | `Odometry` | Wheel odometry |
| `/joint_states` | `JointState` | Wheel joint states |
| `/scan` | `LaserScan` | RPLidar scan |
| `/camera/image_raw` | `Image` | ESP32-CAM stream |
| `/imu` | `Imu` | IMU data (from ESP32 via hw_interface) |
| `/battery_state` | `BatteryState` | Battery (from ESP32 via hw_interface) |
| `/cmd_vel_web` | `TwistStamped` | Web UI joystick commands (priority 20) |

## Dependencies

- `pizibot_description` — robot URDF
- `pizibot_hw_interface` — hardware interface plugin
- `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster` — ros2_control
- `rplidar_ros` — RPLidar driver
- `twist_mux` — velocity multiplexer

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache-2.0
