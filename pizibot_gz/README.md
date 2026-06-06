# Pizibot GZ (Gazebo Harmonic)

ROS 2 Jazzy simulation package for the Pizibot robot using **Gazebo Harmonic** (gz-sim).

## Description

This package is the successor of `pizibot_gazebo` (based on Gazebo Classic, which is end-of-life). It provides complete simulation of the Pizibot robot with:

- **Gazebo Harmonic (gz-sim)** — next-generation simulator
- **ROS 2 Jazzy** — compatible ROS 2 distribution
- **Robot Control** — differential drive controllers and state publishers
- **ROS-Gazebo Bridges** — complete integration via `ros_gz_bridge`

## Dependency Installation

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

## Usage

### Launch the simulation

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch pizibot_gz launch_sim.launch.py
```

With the web remote interface:

```bash
ros2 launch pizibot_gz launch_sim.launch.py enable_remote_ui:=true
```

Open `http://localhost:3000` in a browser or on a smartphone on the same LAN.

### Launch with empty world (simulator testing)

For testing the simulator and spawner without a world or ROS bridges:

```bash
ros2 launch pizibot_gz launch_sim_empty.launch.py
```

This minimal launch only starts:

- Gazebo Harmonic simulator with an empty world
- Robot state publisher (publishes URDF)
- Robot spawner (spawns the robot entity)

**Use case**: debug simulator physics, test entity spawning, or verify URDF without the overhead of world content and Gazebo-ROS bridges.

### Launch with a different world

```bash
ros2 launch pizibot_gz launch_sim.launch.py \
  gazebo_world:=$(pwd)/install/pizibot_gz/share/pizibot_gz/worlds/world_test2.world
```

### Launch arguments

| Argument          | Default                        | Description                                          |
|-------------------|--------------------------------|------------------------------------------------------|
| `gazebo_world`    | `industrial-warehouse.sdf`     | Full path to the SDF world file                      |
| `enable_remote_ui`| `false`                        | Launch rosbridge + web_video_server + HTTP server    |

## Package Structure

```text
pizibot_gz/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── dataset_saver.cpp           # Node for dataset collection (RGB + masks)
├── launch/
│   ├── launch_sim.launch.py        # Main simulation launch file
│   └── dataset_save.launch.py      # Dataset collection launch file
├── params/
│   ├── sim_controllers.yaml        # Controller parameters
│   └── twist_mux.yaml              # Velocity command multiplexer
├── worlds/
│   ├── industrial-warehouse.sdf    # Industrial warehouse (default)
│   ├── world_minimal.sdf           # Minimal world for quick testing
│   ├── world_test1.world           # Test world 1
│   └── world_test2.world           # Test world 2
└── rviz/
    └── sim_viz.rviz                # RViz configuration
```

## Differences from pizibot_gazebo

| Aspect           | pizibot_gazebo (Gazebo Classic) | pizibot_gz (Gazebo Harmonic) |
|------------------|---------------------------------|------------------------------|
| Simulator        | Gazebo Classic 11 (EOL)         | Gazebo Harmonic 8.x          |
| ROS 2            | Humble                          | Jazzy                        |
| Package launcher | gazebo_ros                      | ros_gz_sim                   |
| Entity spawner   | gazebo_ros/spawn_entity.py      | ros_gz_sim/create            |
| Bridging         | Gazebo Plugin                   | ros_gz_bridge (external)     |

## Available Nodes

- `robot_state_publisher` — publishes robot state
- `gz_sim` — Gazebo Harmonic simulator
- `ros_gz_bridge` — ROS 2 ↔ Gazebo bridge
- `ros_gz_image` — bridge for camera images (RGB and semantic segmentation)
- `twist_mux` — velocity command multiplexer (outputs `TwistStamped`)
- `twist_stamped_to_twist` — converts `TwistStamped` → `Twist` for the Gazebo bridge
- `dataset_saver` — saves RGB images and binary masks for dataset collection

## ROS Topics

### Published by simulation

| Topic                          | Type          | Description                           |
| ------------------------------ | ------------- | ------------------------------------- |
| `/scan`                        | `LaserScan`   | LiDAR scan (360 samples, 10 Hz)       |
| `/camera/image_raw`            | `Image`       | Raw camera (1600×1200, 15 Hz)         |
| `/camera/camera_info`          | `CameraInfo`  | Camera calibration info               |
| `/camera/semantic/labels_map`  | `Image`       | Semantic segmentation labels          |
| `/camera/semantic/colored_map` | `Image`       | Semantic segmentation visualization   |
| `/imu`                         | `Imu`         | IMU data                              |
| `/joint_states`                | `JointState`  | Wheel joint states                    |
| `/odom`                        | `Odometry`    | Robot odometry                        |
| `/tf`                          | `TFMessage`   | Frame transforms                      |

### Velocity command topics

| Topic          | Type            | Description                                       |
| -------------- | --------------- | ------------------------------------------------- |
| `/cmd_vel`     | `TwistStamped`  | Navigation input (priority 19)                    |
| `/cmd_vel_joy` | `TwistStamped`  | Joystick input (priority 21)                      |
| `/cmd_vel_key` | `TwistStamped`  | Keyboard input (priority 22)                      |
| `/cmd_vel_web` | `TwistStamped`  | Web UI input (priority 20)                        |
| `/cmd_vel_out` | `TwistStamped`  | twist_mux output → twist_stamped_to_twist         |
| `/cmd_vel_gz`  | `Twist`         | Gazebo bridge input (converted from TwistStamped) |

## Joystick Teleoperation

### Launch teleoperation

```bash
ros2 launch pizibot_teleop joystick_teleop.launch.py
```

### Gamepad configuration (generic USB)

- **Enable**: LB (button 4)
- **Turbo**: RB (button 5)
- **Linear motion**: Left stick Y-axis — Normal: 0.3 m/s · Turbo: 0.8 m/s
- **Rotation**: Left stick X-axis — Normal: 0.75 rad/s · Turbo: 2.0 rad/s

Configurable parameters in: `pizibot_teleop/params/joystick.yaml`

## Dataset Collection

The `dataset_saver` node synchronizes RGB and semantic images. Images are saved only when the robot has moved sufficiently (distance > 0.05 m or rotation > 30°).

### Launch dataset collection

```bash
ros2 launch pizibot_gz dataset_save.launch.py
```

### With custom parameters

```bash
ros2 launch pizibot_gz dataset_save.launch.py \
  label:=1 \
  topic_rgb:=/camera/image_raw \
  topic_sem:=/camera/semantic/labels_map \
  dataset_path:=/path/to/dataset
```

### Parameters

| Parameter      | Default                            | Description                                          |
|----------------|------------------------------------|------------------------------------------------------|
| `label`        | `1`                                | Semantic class label (→ black in mask)               |
| `topic_rgb`    | `/camera/image_raw`                | ROS topic for the RGB image                          |
| `topic_sem`    | `/camera/semantic/labels_map`      | ROS topic for the semantic image                     |
| `dataset_path` | `dataset`                          | Output folder (creates `images/` and `masks/`)       |

### Output structure

```text
dataset/
├── images/
│   ├── 000000.png
│   └── ...
└── masks/
    ├── 000000.png
    └── ...
```

## Simulation Parameters

### Robot dynamics (`gazebo_config.xacro`)

- Max linear acceleration: ±1.0 m/s²
- Max angular acceleration: ±2.0 rad/s²
- Odometry publication: 50 Hz

### Sensors

- **LiDAR**: 360 samples, 10 Hz, range 0.15–12 m
- **Camera**: 1600×1200, 15 Hz, FOV 1.089 rad
- **Wheels**: separation 0.273 m, diameter 0.08 m

## Available Worlds

- **industrial-warehouse.sdf** (default) — industrial warehouse with shelves and obstacles
- **world_minimal.sdf** — minimal world for quick testing
- **world_test1.world** — simple test world
- **world_test2.world** — world with obstacles

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache-2.0
