#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "pizibot_hardware/ADS1115.h"
#include <chrono>

using namespace std::chrono_literals;

class BatteryStatePublisher : public rclcpp::Node
{
public:
    BatteryStatePublisher()
        : Node("battery_state_publisher_node"), ads1115_()
    {

        // Create publisher on the battery_state topic
        battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

        // Set up a timer to publish battery state at 5-second intervals
        timer_ = this->create_wall_timer(5s, std::bind(&BatteryStatePublisher::publishBatteryState, this));
    }


private:
    void publishBatteryState()
    {
        // Open ADS1115 connection and check for successful initialization
        while (!ads1115_.open())
        {
            RCLCPP_WARN(get_logger(), "Failed to open ADS1115 connection, retrying...");
            rclcpp::sleep_for(100ms);
        }

        // Configure the ADS1115
        ads1115_.configure();

        // Read voltage from ADS1115
        float readVoltage = ads1115_.readVoltage();
        ads1115_.close();
        float batteryVoltage = readVoltage * 5.0;               // Adjust scaling factor as per ADS1115 configuration
        float batteryPercentage = (0.5 * batteryVoltage) - 3.2; // Adjust scaling formula as needed
        batteryPercentage = std::clamp(batteryPercentage, 0.0f, 1.0f); // Constrain percentage between 0-1

        // Populate message with battery state data
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = batteryVoltage;
        message.percentage = batteryPercentage;

        // Publish the message
        RCLCPP_INFO(get_logger(), "Publishing Battery Voltage: %.2f V, Percentage: %.2f%%", batteryVoltage, batteryPercentage * 100);
        battery_state_pub_->publish(message);
    }
    void publishZero() {
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = 0.0;
        message.percentage = 0.0;

        // Publish the message
        RCLCPP_INFO(get_logger(), "Publishing Battery Voltage: %.2f V, Percentage: %.2f%%", 0.0, 0.0);
        battery_state_pub_->publish(message);
    }

    ADS1115 ads1115_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Run the BatteryStatePublisher node
    rclcpp::spin(std::make_shared<BatteryStatePublisher>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
