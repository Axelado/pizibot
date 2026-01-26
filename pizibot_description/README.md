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
ros2 run rviz2 rviz2 -d install/pizibot_description/share/pizibot_description/rviz/config.rviz
```

## Package Structure

```
pizibot_description/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata
├── launch/
│   └── rsp.launch.py        # Robot State Publisher launch file
├── urdf/
│   ├── pizibot.urdf.xacro   # Main robot description
│   ├── gazebo_config.xacro  # Gazebo-specific configuration
│   └── ...                  # Other URDF components
├── meshes/
│   ├── base/                # Base mesh files
│   ├── sensors/             # Sensor mesh files
│   └── ...
└── rviz/
    └── config.rviz          # RViz visualization configuration
```

## URDF Components

The robot description includes:

- **Base Link** - Main robot body
- **Drive Wheels** - Left and right wheel joints and links
- **Caster Wheel** - Caster wheel for stability
- **Sensors**:
  - LiDAR (RPLiDAR A1)
  - RGB Camera
  - Segmentation Camera (semantic segmentation for simulation)
  - IMU
  - Odometry computation

## Key Parameters

### Robot Dimensions
- Base width: 0.273 m
- Wheel diameter: 0.08 m
- Wheel separation: 0.273 m
- Overall height: ~0.15 m

### Coordinate Frames
- `base_link` - Robot base reference frame
- `base_footprint` - Ground projection of the base
- `lidar_link` - LiDAR sensor frame
- `camera_link` - Camera sensor frame
- `camera_link_optical` - Camera optical frame (for image data)
- `imu_link` - IMU sensor frame
- `left_wheel_link`, `right_wheel_link` - Wheel frames

## Simulation Integration

This package integrates seamlessly with:
- **Gazebo Harmonic** (`pizibot_gz`) - For simulation
- **Nav2** - For autonomous navigation
- **SLAM Toolbox** - For mapping

## Dependencies

- `urdf` - Universal Robotics Description Format
- `xacro` - URDF macro language
- `robot_state_publisher` - Publishes robot description and transforms
- `tf2` - Transform library

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache 2.0
