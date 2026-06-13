/**
 * @file initial_pose_publisher.cpp
 * @brief ROS 2 node that publishes an initial pose estimate for AMCL localization
 *
 * This node waits until AMCL subscribes to /initialpose, then publishes a
 * PoseWithCovarianceStamped message on that topic for 3 seconds at 10 Hz to
 * ensure AMCL receives and processes the initial pose.
 *
 * @author Axel NIATO
 * @date January 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @class InitialPosePublisher
 * @brief Publishes an initial pose estimate to /initialpose topic
 *
 * This node reads an initial pose parameter [x, y, yaw] and waits until AMCL
 * subscribes to /initialpose before publishing it for AMCL localization. The
 * pose is then published repeatedly for 3 seconds to ensure reliable delivery.
 */
class InitialPosePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor that initializes the node and starts publishing
     *
     * Declares and reads the initial_pose parameter, validates it, and
     * starts a timer to publish the pose for 3 seconds.
     */
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

        // Set the frame_id to 'map' for localization
        msg_.header.frame_id = "map";
        
        // Set position from parameters
        msg_.pose.pose.position.x = pose[0];
        msg_.pose.pose.position.y = pose[1];
        msg_.pose.pose.position.z = 0.0;

        // Convert yaw to quaternion
        double yaw = pose[2];
        msg_.pose.pose.orientation.x = 0.0;
        msg_.pose.pose.orientation.y = 0.0;
        msg_.pose.pose.orientation.z = sin(yaw / 2.0);
        msg_.pose.pose.orientation.w = cos(yaw / 2.0);

        // Set default covariance values for AMCL
        // Covariance matrix is 6x6, stored row-major
        msg_.pose.covariance[0] = 0.25;   // x variance
            msg_.pose.covariance[7] = 0.25;   // y variance
        msg_.pose.covariance[35] = 0.0685; // yaw variance

        // Check at 10 Hz whether AMCL has subscribed yet, then publish for 3 seconds
        timer_ = this->create_wall_timer(
            100ms, std::bind(&InitialPosePublisher::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for a subscriber on /initialpose...");
    }

private:
    /**
     * @brief Timer callback that waits for AMCL, then publishes the pose
     *
     * Until a subscriber is present on /initialpose, the callback does
     * nothing. Once AMCL is ready, the pose is published for 3 seconds,
     * after which the node shuts down.
     */
    void timer_callback()
    {
        if (!publishing_) {
            if (publisher_->get_subscription_count() == 0) {
                return;
            }
            publishing_ = true;
            start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                        "AMCL is ready: publishing initial pose x=%.2f, y=%.2f, yaw=%.2f rad (for 3 seconds)",
                        pose_x_, pose_y_, pose_yaw_);
        }

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
    bool publishing_ = false;
    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_yaw_ = 0.0;
};

/**
 * @brief Main function that initializes and spins the node
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}