#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
    : Node("initial_pose_publisher")
    {
        this->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0});
        auto pose = this->get_parameter("initial_pose").as_double_array();

        if (pose.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "initial_pose parameter must be [x, y, yaw]");
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

        msg_.header.frame_id = "map";
        msg_.pose.pose.position.x = pose[0];
        msg_.pose.pose.position.y = pose[1];
        msg_.pose.pose.position.z = 0.0;

        double yaw = pose[2];
        msg_.pose.pose.orientation.z = sin(yaw / 2.0);
        msg_.pose.pose.orientation.w = cos(yaw / 2.0);

        // Optionally set a default covariance
        msg_.pose.covariance[0] = 0.25;   // x
        msg_.pose.covariance[7] = 0.25;   // y
        msg_.pose.covariance[35] = 0.0685; // yaw

        // Start timer to publish for 3 seconds at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&InitialPosePublisher::timer_callback, this)
        );
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "InitialPosePublisher started, publishing for 3 seconds...");
    }

private:
    void timer_callback()
    {
        auto now = this->now();
        msg_.header.stamp = now;
        publisher_->publish(msg_);
        if ((now - start_time_).seconds() >= 3.0) {
            RCLCPP_INFO(this->get_logger(), "Finished publishing initial pose.");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    geometry_msgs::msg::PoseWithCovarianceStamped msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}