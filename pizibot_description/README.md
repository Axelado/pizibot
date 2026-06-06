# Pizibot Description

ROS 2 Jazzy URDF and robot model package for the Pizibot robot.

## Description

This package contains the robot description files for the Pizibot two-wheeled mobile robot, including:

- **URDF/Xacro Files** - Complete robot kinematics and dynamics model
- **Meshes** - 3D mesh files for visualization
- **RViz Configuration** - Pre-configured RViz setups for visualization
- **ROS 2 Jazzy** - Fully compatible with ROS 2 Jazzy

## Migration Status

✅ This package is **fully migrated to ROS 2 Jazzy**.

## Usage

### Launch Robot State Publisher

To publish the robot description and transforms:

```bash
ros2 launch pizibot_description rsp.launch.py
```

**Optional parameters:**

- `use_sim_time` (default: false) - Use simulation time if true

```bash
ros2 launch pizibot_description rsp.launch.py use_sim_time:=true
```

### Visualize with RViz

```bash
ros2 run rviz2 rviz2 -d install/pizibot_description/share/pizibot_description/rviz/viz_bot.rviz
```

## Package Structure

```text
pizibot_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── rsp.launch.py
├── urdf/
│   ├── pizibot.urdf.xacro      # Main robot description (top-level entry point)
│   ├── pizibot_macro.xacro     # Base link, chassis, wheels, casters
│   ├── dimensions.xacro        # All physical dimensions and sensor properties
│   ├── lidar.xacro             # RPLidar A1 link, joint, and Gazebo plugin
│   ├── camera.xacro            # Camera link, joint, and Gazebo plugin
│   ├── control.xacro           # ros2_control configuration
│   ├── gazebo_config.xacro     # Gazebo-specific settings (IMU, odometry)
│   ├── inertial_macros.xacro   # Inertia helper macros
│   └── material_colors.xacro   # Visual material definitions
├── meshes/
│   ├── robot_chassis.stl       # Robot body
│   ├── wheel.stl               # Shared mesh for both drive wheels
│   ├── rplidar_a1.stl          # RPLidar A1 visual
│   └── tire_80mm.stl           # 80 mm tire (reference mesh)
└── rviz/
    └── viz_bot.rviz            # RViz preset for robot model visualization
```

## URDF Architecture

Visual and collision geometry are separated into distinct links:

| Link | Role |
| ---- | ---- |
| `base_link` | Robot reference origin |
| `base_footprint` | Ground-plane projection |
| `chassis` | Collision geometry + inertia for the body |
| `chassis_visual` | Visual mesh (`robot_chassis.stl`) — fixed to `base_link` |
| `left_wheel` / `right_wheel` | Drive wheel collision, inertia, and visual (`wheel.stl`) |
| `front_caster` / `back_caster` | Sphere caster collision and visual |
| `front_caster_cylinder` / `back_caster_cylinder` | Caster stem collision only |
| `laser_frame` | LiDAR collision geometry + inertia |
| `laser_visual` | LiDAR visual mesh (`rplidar_a1.stl`) — fixed to chassis |
| `camera_link` | Camera sensor frame (directly on chassis, no support link) |
| `camera_link_optical` | Camera optical frame |

## Key Parameters (`dimensions.xacro`)

### Robot Body

| Parameter | Value |
| --------- | ----- |
| `robot_diameter` | 0.290 m |
| `robot_height` | 0.103 m (with wheels) |
| `robot_mass` | 1 kg |

### Drive Wheels

| Parameter | Value |
| --------- | ----- |
| `wheel_diameter` | 0.08 m |
| `wheel_width` | 0.04 m |
| `wheel_separation` | 0.29 m |
| `wheel_y_offset` | 0.01969 m |
| `max_wheel_velocity` | 4.6 rad/s (≈ 43.9 RPM) |
| `max_wheel_effort` | 40 Nm |

### Caster Wheels

| Parameter | Value |
| --------- | ----- |
| `caster_x_offset` | 0.09 m |
| `caster_diameter` | 0.016 m |
| `caster_mass` | 0.1 kg |
| `ground_clearance` | `0.057 − caster_diameter/2` ≈ 0.049 m |

### RPLidar A1

| Parameter | Value |
| --------- | ----- |
| `lidar_x_offset` | 0.055 m |
| `lidar_diameter` | 0.098 m |
| `lidar_height` | 0.062 m |
| `lidar_mass` | 0.1 kg |
| `lidar_update_rate` | 10 Hz |
| `lidar_min_range` / `lidar_max_range` | 0.15 m / 12.0 m |

### Camera

| Parameter | Value |
| --------- | ----- |
| `camera_x_offset` | `robot_diameter / 2` = 0.145 m |
| `camera_z_offset` | 0.080 m |
| `camera_horizontal_fov` | 1.089 rad |
| `camera_image_width` / `camera_image_height` | 1600 × 1200 px |
| `camera_update_rate` | 15 Hz |
| `camera_clip_near` / `camera_clip_far` | 0.05 m / 8.0 m |

## Coordinate Frames

| Frame | Description |
| ----- | ----------- |
| `base_link` | Robot reference origin |
| `base_footprint` | Ground projection |
| `chassis` | Body collision/inertia frame |
| `chassis_visual` | Body visual frame |
| `left_wheel` / `right_wheel` | Drive wheel frames |
| `front_caster` / `back_caster` | Caster sphere frames |
| `laser_frame` | LiDAR sensor frame |
| `laser_visual` | LiDAR visual frame |
| `camera_link` | Camera sensor frame |
| `camera_link_optical` | Camera optical frame (image data) |

## Simulation Integration

This package integrates seamlessly with:

- **Gazebo Harmonic** (`pizibot_gz`) — For simulation
- **Nav2** — For autonomous navigation
- **SLAM Toolbox** — For mapping

## Dependencies

- `urdf` — Universal Robotics Description Format
- `xacro` — URDF macro language
- `robot_state_publisher` — Publishes robot description and transforms
- `tf2` — Transform library

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache 2.0
