# Pizibot Teleop

ROS 2 Jazzy teleoperation package for the Pizibot robot. Supports both keyboard and joystick-based control.

## Description

This package provides teleoperation capabilities for the Pizibot robot with:

- **Keyboard Teleoperation** - AZERTY keyboard layout support
- **Joystick Teleoperation** - Generic USB gamepad support
- **ROS 2 Jazzy** - Compatible with ROS 2 Jazzy

## Migration Status

✅ This package is **fully migrated to ROS 2 Jazzy**.

## Installation

Ensure you have ROS 2 Jazzy installed and sourced:

```bash
source /opt/ros/jazzy/setup.bash
```

## Usage

### Keyboard Teleoperation

Launch keyboard-based teleoperation with AZERTY layout:

```bash
ros2 launch pizibot_teleop keyboard_teleop.launch.py
```

**Keyboard Controls:**
```
Moving around:
   a    z    e
   q    s    d
   w    c

For Holonomic mode (strafing), hold down the shift key:
   A    Z    E
   Q    S    D
   W    C

t : up (+z)
g : down (-z)

r/f : increase/decrease max speeds by 10%
t/g : increase/decrease only linear speed by 10%
y/h : increase/decrease only angular speed by 10%

CTRL-C to quit
```

### Joystick Teleoperation

Launch joystick-based teleoperation:

```bash
ros2 launch pizibot_teleop joystick_teleop.launch.py
```

**Gamepad Configuration (generic USB):**
- **Enable**: LB (button 4)
- **Turbo**: RB (button 5)
- **Linear motion**: Left stick Y-axis
  - Normal: 0.3 m/s
  - Turbo: 0.8 m/s
- **Rotation**: Left stick X-axis
  - Normal: 0.75 rad/s
  - Turbo: 2.0 rad/s

Configurable parameters in: `params/joystick.yaml`

## ROS Topics

### Published Topics
- `/cmd_vel_key` - Keyboard velocity commands
- `/cmd_vel_joy` - Joystick velocity commands

### Subscribed Topics
- `/joy` - Joystick input (from `joy_node`)

## Configuration Files

- `params/joystick.yaml` - Joystick button mappings and speed parameters
- `launch/keyboard_teleop.launch.py` - Keyboard teleoperation launcher
- `launch/joystick_teleop.launch.py` - Joystick teleoperation launcher

## Dependencies

- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - ROS message types for geometry
- `teleop_twist_keyboard_for_azerty` - AZERTY keyboard teleoperation node
- `joy` - Joystick driver

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache 2.0
