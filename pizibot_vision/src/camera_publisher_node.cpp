/**
 * @file camera_publisher_node.cpp
 * @brief Camera Publisher Node for Pizibot - ROS 2 Jazzy compatible
 * 
 * This node captures images from a connected USB camera and publishes them
 * as ROS 2 messages to a configurable topic. It provides:
 * 
 * Features:
 *   - Automatic camera detection with fallback to indices 0-9
 *   - Configurable camera index, resolution (width/height), and frame rate (FPS)
 *   - Efficient image-to-ROS-message conversion using cv_bridge
 *   - Throttled logging to prevent message spam on errors
 *   - Clean resource management with RAII principles
 * 
 * Parameters:
 *   - camera_index (int, default 0): Camera device index to use
 *   - image_topic (string, default "camera/image_raw"): Topic for image publishing
 *   - width (int, default 640): Image width in pixels
 *   - height (int, default 480): Image height in pixels
 *   - fps (int, default 30): Capture and publish frame rate
 *   - frame_id (string, default "camera_link"): TF frame ID for image header
 * 
 * Published Topics:
 *   - camera/image_raw (sensor_msgs/Image): Raw camera frames in BGR8 format
 * 
 * Usage:
 *   ros2 launch pizibot_vision camera.launch.py
 *   ros2 launch pizibot_vision camera.launch.py camera_index:=1 fps:=15
 * 
 * Dependencies:
 *   - ROS 2 Jazzy or compatible
 *   - OpenCV (cv::VideoCapture for camera access)
 *   - cv_bridge (image format conversion)
 *   - rclcpp (ROS 2 C++ client library)
 *   - sensor_msgs (Image message type)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor: Initialize the camera publisher node
     * 
     * Declares ROS parameters, initializes the camera, and sets up the
     * publishing timer. If camera initialization fails, logs an error but
     * allows the node to continue (useful for debugging).
     */
    CameraPublisher()
    : Node("camera_publisher")
    {
        // Declare ROS parameters with their default values
        this->declare_parameter<int>("camera_index", 0);
        this->declare_parameter<std::string>("image_topic", "camera/image_raw");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("frame_id", "camera_link");

        // Retrieve parameters from parameter server
        camera_index_ = this->get_parameter("camera_index").as_int();
        image_topic_ = this->get_parameter("image_topic").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Create publisher for image messages
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);

        // Initialize camera device
        if (!initializeCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera. Node will not publish images.");
            return;
        }

        // Create timer for periodic image capture and publishing
        auto period = std::chrono::milliseconds(1000 / std::max(fps_, 1));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&CameraPublisher::captureAndPublish, this)
        );

        RCLCPP_INFO(this->get_logger(), 
            "CameraPublisher started on topic '%s' (camera %d, %dx%d @ %d FPS)",
            image_topic_.c_str(), camera_index_, width_, height_, fps_);
    }

    /**
     * @brief Destructor: Clean up camera resources
     * 
     * Releases the camera device and logs the shutdown.
     */
    ~CameraPublisher()
    {
        if (cap_.isOpened()) {
            cap_.release();
            RCLCPP_INFO(this->get_logger(), "Camera released");
        }
    }

private:
    /**
     * @brief Initialize the camera device
     * 
     * Attempts to open the camera at the specified index. If that fails,
     * tries indices 0-9 as fallback. Configures camera properties (resolution, FPS).
     * 
     * @return true if camera was successfully initialized, false otherwise
     */
    bool initializeCamera()
    {
        // Try to open the camera with the given index
        cap_.open(camera_index_);
        
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), 
                "Unable to open camera %d, trying other indices...", camera_index_);
            
            // Try other camera indices (0-9) as fallback
            for (int idx = 0; idx < 10; ++idx) {
                if (idx == camera_index_) continue;
                
                cap_.open(idx);
                if (cap_.isOpened()) {
                    camera_index_ = idx;
                    RCLCPP_INFO(this->get_logger(), "Camera opened with index %d", idx);
                    break;
                }
            }
        }

        // Check if camera was successfully opened
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), 
                "Unable to open any camera (tried indices 0-9)");
            return false;
        }

        // Configure camera properties
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);

        return true;
    }

    /**
     * @brief Capture an image frame and publish it to ROS
     * 
     * This method is called periodically by the timer. It:
     * 1. Captures a frame from the camera
     * 2. Validates the frame
     * 3. Converts it to a ROS Image message
     * 4. Publishes the message to the configured topic
     * 
     * Empty frames are logged with throttling to avoid spam.
     */
    void captureAndPublish()
    {
        // Capture frame from camera
        cv::Mat frame;
        cap_ >> frame;

        // Validate frame
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Empty image captured from camera");
            return;
        }

        // Create ROS message header with timestamp and frame ID
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id_;

        // Convert OpenCV image (BGR8) to ROS Image message
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        
        // Publish the image message
        publisher_->publish(*msg);
    }

    // ==========================================================================
    // Member Variables
    // ==========================================================================
    
    // ROS 2 members
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // OpenCV members
    cv::VideoCapture cap_;
    
    // Configuration parameters
    int camera_index_;
    std::string image_topic_;
    int width_;
    int height_;
    int fps_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the camera publisher node
    auto node = std::make_shared<CameraPublisher>();
    
    // Spin the node (process callbacks and timers)
    rclcpp::spin(node);
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    return 0;
}
