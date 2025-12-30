# Pizibot GZ (Gazebo Harmonic)

ROS 2 Jazzy simulation package for the Pizibot robot using **Gazebo Harmonic** (gz-sim).

## Description

This package is the successor of `pizibot_gazebo` (based on Gazebo Classic, which is end-of-life). It provides complete simulation of the Pizibot robot with:

- **Gazebo Harmonic (gz-sim)** - Next-generation simulator
- **ROS 2 Jazzy** - Compatible ROS 2 distribution
- **Robot Control** - Differential drive controllers and state publishers
- **ROS-Gazebo Bridges** - Complete integration via `ros_gz_bridge`

## Dependency Installation

```bash
sudo apt install -y ros-jazzy-ros-gz
sudo apt install -y ros-jazzy-ros-gz-sim
sudo apt install -y ros-jazzy-ros-gz-bridge
```

## Usage

### Launch the simulation

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch pizibot_gz launch_sim.launch.py
```

### Launch with empty world (simulator testing)

For testing the simulator and spawner without a world or ROS bridges:

```bash
ros2 launch pizibot_gz launch_sim_empty.launch.py
```

This minimal launch only starts:
- Gazebo Harmonic simulator with an empty world
- Robot state publisher (publishes URDF)
- Robot spawner (spawns the robot entity)

**Use case**: Debug simulator physics, test entity spawning, or verify URDF without the overhead of world content and Gazebo-ROS bridges.

### Launch with a different world

```bash
ros2 launch pizibot_gz launch_sim.launch.py gazebo_world:=$(pwd)/install/pizibot_gz/share/pizibot_gz/worlds/world_test2.world
```

### Use a prefix for multi-robot

```bash
ros2 launch pizibot_gz launch_sim.launch.py prefix:=robot1_
```

## Package Structure

```
pizibot_gz/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata
├── src/
│   └── yaml_modifier_node.cpp  # Node for modifying YAML parameters
├── launch/
│   └── launch_sim.launch.py # Main launch file
├── params/
│   ├── sim_controllers.yaml # Controller parameters
│   └── twist_mux.yaml       # Velocity command multiplexer
├── worlds/
│   ├── world_test1.world    # Test world 1 (SDF)
│   └── world_test2.world    # Test world 2 (SDF)
└── rviz/
    └── sim_viz.rviz         # RViz configuration
```

## Differences from pizibot_gazebo

| Aspect | pizibot_gazebo (Gazebo Classic) | pizibot_gz (Gazebo Harmonic) |
|--------|--------------------------------|------------------------------|
| Simulator | Gazebo Classic 11 (EOL) | Gazebo Harmonic 8.x |
| ROS 2 | Humble | Jazzy |
| Package launcher | gazebo_ros | ros_gz_sim |
| Entity spawner | gazebo_ros/spawn_entity.py | ros_gz_sim/create |
| Bridging | Gazebo Plugin | ros_gz_bridge (external) |
| Performance | Legacy | Improved |

## Available Nodes

- `robot_state_publisher` - Publishes robot state
- `gz_sim` - Gazebo Harmonic simulator
- `ros_gz_bridge` - ROS 2 ↔ Gazebo bridge
- `ros_gz_image` - Bridge for camera images
- `twist_mux` - Velocity command multiplexer (optional, currently disabled)

## ROS Topics

### Published by simulation
- `/scan` - LiDAR scan (360 samples, 10 Hz)
- `/camera/image_raw` - Raw camera image (1600x1200, 15 Hz)
- `/camera/camera_info` - Camera calibration info
- `/imu` - IMU data
- `/joint_states` - Joint states (wheels)
- `/odom` - Robot odometry
- `/tf` - Frame transforms

### For control
- `/cmd_vel_gz` - Direct velocity command to Gazebo plugin (main input)
- `/cmd_vel` - Secondary velocity command (navigation)
- `/cmd_vel_joy` - Velocity command from joystick

## Joystick Teleoperation

### Launch teleoperation
```bash
ros2 launch pizibot_teleop joystick_teleop.launch.py
```

### Gamepad configuration (generic USB)
- **Enable**: LB (button 4)
- **Turbo**: RB (button 5)
- **Linear motion**: Left stick Y-axis
  - Normal: 0.3 m/s
  - Turbo: 0.8 m/s
- **Rotation**: Left stick X-axis
  - Normal: 0.75 rad/s
  - Turbo: 2.0 rad/s

Configurable parameters in: `pizibot_teleop/params/joystick.yaml`

## Available Worlds

- **world_test1.world** (default) - Simple test world
- **world_test2.world** - World with obstacles

## Simulation Parameters

### Robot Dynamics (gazebo_config.xacro)
- Max linear acceleration: ±1.0 m/s²
- Max angular acceleration: ±2.0 rad/s²
- Odometry publication frequency: 50 Hz

### Sensor Parameters
- **LiDAR**: 360 samples, 10 Hz, range 0.15-12m
- **Camera**: 1600x1200, 15 Hz, FOV 1.089 rad
- **Wheels**: Separation 0.273m, diameter 0.08m

## Migration Notes

This package completely replaces `pizibot_gazebo` as part of the **Pizibot migration to ROS 2 Jazzy and Gazebo Harmonic**.

### Package Status
- ✅ This package (`pizibot_gz`) is fully migrated to **ROS 2 Jazzy** and **Gazebo Harmonic**
- ✅ `pizibot_description` - Migrated
- ✅ `pizibot_teleop` - Migrated
- ⏳ Other packages are pending migration (see main README for status)

### Migration Guide

To migrate from the old `pizibot_gazebo`:

1. Update the launch file in your scripts
2. Replace `pizibot_gazebo` with `pizibot_gz`
3. Adapt world files if necessary (switch from `.world` to `.sdf` if needed)
4. Check bridged topics in the launch file
5. Ensure your ROS 2 distribution is set to Jazzy or compatible

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache 2.0
