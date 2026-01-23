# Pizibot Vision

**ROS 2 Jazzy compatible** computer vision package providing camera image capture and publishing for the Pizibot robot.

## Overview

`pizibot_vision` provides a robust camera publisher node that:
- Captures images from connected USB cameras
- Publishes frames as `sensor_msgs/Image` to a configurable ROS 2 topic
- Automatically falls back to alternative camera indices (0-9) if the requested device is unavailable
- Supports configurable resolution, frame rate (FPS), topic name, and TF frame ID
- Uses OpenCV for video capture and `cv_bridge` for ROS integration
- Efficient throttled logging to avoid message spam on repeated errors

This package forms the foundation for any computer vision tasks on the Pizibot robot, including object detection, visual SLAM, gesture recognition, or face detection.

## Build & Install

### Prerequisites
- ROS 2 Jazzy installation
- OpenCV development libraries
- cv_bridge for Jazzy

### Build Instructions

```bash
cd ~/ROS/pizi_ws

# Install system and ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select pizibot_vision

# Source the setup file
source install/setup.bash
```

## Usage

### Basic Launch

Start the camera publisher with default parameters:

```bash
ros2 launch pizibot_vision camera.launch.py
```

This will:
1. Attempt to open camera at index 0
2. Capture images at 640×480 at 30 FPS
3. Publish to `camera/image_raw` topic
4. Use `camera_link` as the TF frame ID

### Custom Parameters

Override parameters at launch time:

```bash
ros2 launch pizibot_vision camera.launch.py \
  camera_index:=1 \
  fps:=15 \
  width:=1280 \
  height:=720 \
  image_topic:="camera/high_res" \
  frame_id:="front_camera"
```

### Verify Operation

In another terminal, check if images are being published:

```bash
ros2 topic echo /camera/image_raw --field data | head -5
```

Or use a visualization tool:

```bash
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

## Configuration

### ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_index` | int | `0` | Camera device index (0-9) to open |
| `image_topic` | string | `camera/image_raw` | ROS topic name for image publishing |
| `width` | int | `640` | Image width in pixels |
| `height` | int | `480` | Image height in pixels |
| `fps` | int | `30` | Capture and publish frame rate |
| `frame_id` | string | `camera_link` | TF frame ID for image header |

### Configuration Files

- **`config/camera_params.yaml`**: Default parameter values for the camera publisher

## Topics

### Published Topics

- **`/camera/image_raw`** (`sensor_msgs/Image`)
  - Raw camera frames in BGR8 format
  - Stamped with the current ROS time and configured frame ID
  - Published at the configured FPS

## Package Structure

```
pizibot_vision/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── config/
│   └── camera_params.yaml      # Default parameter values
├── launch/
│   └── camera.launch.py        # Launch file for camera publisher
├── src/
│   └── camera_publisher_node.cpp  # Main C++ node implementation
└── pizibot_vision/
    └── __init__.py             # Python package initialization
```

## Implementation Details

### Node: CameraPublisher

The `camera_publisher_node` is a C++ ROS 2 node that:

1. **Initialization**
   - Declares configurable ROS parameters
   - Attempts to initialize the camera at the specified index
   - Falls back to indices 0-9 if the primary fails
   - Sets up a wall timer for periodic image capture

2. **Camera Setup**
   - Opens the camera device using OpenCV's `VideoCapture`
   - Configures resolution and frame rate
   - Validates successful initialization

3. **Publishing Loop**
   - Captures frames at a fixed interval (based on configured FPS)
   - Converts OpenCV frames (BGR8) to ROS Image messages
   - Stamps each message with the current ROS time and frame ID
   - Publishes to the configured topic
   - Handles empty frames gracefully with throttled warnings

4. **Cleanup**
   - Releases camera resources on shutdown

## Dependencies

### Build & Run Dependencies
- `rclcpp` - ROS 2 C++ client library
- `sensor_msgs` - Standard sensor message definitions
- `std_msgs` - Standard ROS message types
- `cv_bridge` - OpenCV ↔ ROS image conversion
- `image_transport` - Efficient image publishing framework
- `libopencv-dev` - OpenCV development libraries
- `python3-opencv` - Python OpenCV bindings

### Build Tool Dependencies
- `ament_cmake` - CMake build system for ROS 2
- `ament_cmake_python` - Python support in ament_cmake

## Troubleshooting

### Camera Not Found
- Verify the camera is connected: `ls -la /dev/video*`
- Try different `camera_index` values: `camera_index:=0`, `camera_index:=1`, etc.
- Check camera permissions: `sudo usermod -a -G video $USER`

### No Images Published
- Verify the node is running: `ros2 node list`
- Check for error messages: `ros2 node info /camera_publisher`
- Ensure the topic exists: `ros2 topic list`

### Performance Issues
- Reduce FPS: `fps:=15`
- Lower resolution: `width:=320 height:=240`
- Check system resources: `top`, `nvidia-smi`

## References

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [cv_bridge ROS package](https://github.com/ros-perception/vision_opencv)
- [Pizibot Main README](../README.md)

## Author & License

Author: Axel NIATO (<axelniato@gmail.com>)

License: Apache-2.0
