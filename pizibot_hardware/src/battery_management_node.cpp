#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <pigpio.h>
#include <unistd.h>

class BatteryManagementNode : public rclcpp::Node
{
public:
    BatteryManagementNode()
        : Node("battery_management_node")
    {
        relay_pin_ = 23; // GPIO pin to control the relay
        setup();         // Initialize the relay configuration

        // Create a subscriber to listen to the "battery_state" topic
        battery_subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10, std::bind(&BatteryManagementNode::battery_callback, this, std::placeholders::_1));

        // Set the low battery threshold to 20%
        low_battery_threshold_ = 0.20;

        RCLCPP_INFO(this->get_logger(), "Battery Management Node has started.");
    }

private:
    // Function to initialize GPIO configuration and activate the relay
    void setup()
    {
        while (gpioInitialise() < 0) // Retry pigpio initialization if it fails
        {
            sleep(0.1);
        }
        gpioSetMode(relay_pin_, PI_OUTPUT);
        gpioWrite(relay_pin_, PI_ON);
        gpioTerminate(); // Allow other processes to use GPIO
        RCLCPP_INFO(this->get_logger(), "setup OK");
    }

    void relayOff()
    {
        if (gpioInitialise() > 0) // No loop here since battery messages are received continuously, so the relay can be turned off by future messages
        {
            gpioWrite(relay_pin_, PI_OFF);
            gpioTerminate();
            RCLCPP_INFO(this->get_logger(), "relay OFF");
        }
    }

    void relayOn()
    {
        if (gpioInitialise() > 0) // No loop here since battery messages are received continuously, so the relay can be turned on by future messages
        {
            gpioWrite(relay_pin_, PI_ON);
            gpioTerminate();
            RCLCPP_INFO(this->get_logger(), "relay ON");
        }
    }

    // Callback called when the battery message is received
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        float battery_percentage = msg->percentage;
        RCLCPP_INFO(this->get_logger(), "Battery Percentage: %.2f%%", battery_percentage * 100);

        // Check if the percentage is below the threshold
        if (battery_percentage < low_battery_threshold_)
        {
            RCLCPP_WARN(this->get_logger(), "Battery is low! Current percentage: %.2f%%", battery_percentage * 100);
            relayOff(); // Turn off the relay if the battery is low
        }
        else
        {
            relayOn(); // Turn on the relay if the battery level is sufficient
        }
    }

    // Subscriber for the "battery_state" topic
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;

    // Low battery threshold
    float low_battery_threshold_;

    // Pin for the relay
    int relay_pin_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<BatteryManagementNode>());

    rclcpp::shutdown();
    return 0;
}