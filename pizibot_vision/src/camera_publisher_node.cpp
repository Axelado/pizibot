#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <csignal>
#include <atomic>

std::atomic<bool> stop_signal{false};

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        stop_signal = true;
    }
}

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
    : Node("camera_publisher")
    {
        // Declare parameters for flexibility
        this->declare_parameter<int>("camera_index", 0);
        this->declare_parameter<std::string>("image_topic", "camera/image_raw");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 5);

        int camera_index = this->get_parameter("camera_index").as_int();
        std::string image_topic = this->get_parameter("image_topic").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

        // Try to open the camera with the given index, else try others (0-9)
        bool camera_opened = false;
        int tried_index = camera_index;
        cap_.open(camera_index);
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Unable to open camera %d, trying other indices...", camera_index);
            for (int idx = 0; idx < 10; ++idx) {
                if (idx == camera_index) continue;
                cap_.open(idx);
                if (cap_.isOpened()) {
                    tried_index = idx;
                    camera_opened = true;
                    RCLCPP_INFO(this->get_logger(), "Camera opened with index %d", idx);
                    break;
                }
            }
        } else {
            camera_opened = true;
        }

        if (!camera_opened) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open any camera (tried indices 0-9)");
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / std::max(fps, 1)),
            std::bind(&CameraPublisher::captureAndPublish, this)
        );

        RCLCPP_INFO(this->get_logger(), "CameraPublisher started on topic '%s' (camera %d, %dx%d @ %d FPS)",
                    image_topic.c_str(), tried_index, width, height, fps);
    }

private:
    void captureAndPublish()
    {
        if (stop_signal) {
            RCLCPP_INFO(this->get_logger(), "SIGINT received, shutting down node.");
            rclcpp::shutdown();
            return;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty image captured");
            return;
        }

        // Convert and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    std::signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
