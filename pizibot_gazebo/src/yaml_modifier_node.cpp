#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>


class YamlModifierNode : public rclcpp::Node {
public:
    YamlModifierNode() : Node("yaml_modifier_node") {

        auto file_path_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        file_path_param_desc.description = "The path to the source yaml file!";
        this->declare_parameter("file_path", "", file_path_param_desc);

        auto prefix_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        prefix_param_desc.description = "The prefix to add before frames and joints names!";
        this->declare_parameter("prefix", "", prefix_param_desc);

        std::string file_path = this->get_parameter("file_path").as_string();
        std::string prefix = this->get_parameter("prefix").as_string();
        
        // If the parameters are valid, modify the YAML file
        if (!file_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Modifying YAML file with prefix: '%s'", prefix.c_str());
            modify_yaml_with_prefix(file_path, prefix);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameters: file_path and prefix must  set.");
        }
    }

private:
    // Function to add the prefix to a string
    std::string add_prefix(const std::string& prefix, const std::string& value) {
        return prefix + value;
    }

    // Function to modify the YAML file with the prefix
    void modify_yaml_with_prefix(const std::string& file_path, const std::string& prefix) {
        std::string out_file = file_path.substr(0, file_path.size() - 5) + "_generated.yaml";

        // Si le préfixe est vide, recopier le fichier d'origine
        if (prefix.empty()) {
            std::ifstream src(file_path);
            std::ofstream dst(out_file);
            dst << src.rdbuf();
            src.close();
            dst.close();
            RCLCPP_INFO(this->get_logger(), "Prefix is empty, copied original YAML to '%s' without modification.", out_file.c_str());
            return;
        }

        YAML::Node config = YAML::LoadFile(file_path);

        // Check and modify the parameters by adding the prefix
        if (config["diff_cont"] && config["diff_cont"]["ros__parameters"]) {
            // Add the prefix to base_frame_id
            if (config["diff_cont"]["ros__parameters"]["base_frame_id"]) {
                std::string base_frame_id = config["diff_cont"]["ros__parameters"]["base_frame_id"].as<std::string>();
                config["diff_cont"]["ros__parameters"]["base_frame_id"] = add_prefix(prefix, base_frame_id);
            }

            // Add the prefix to the left wheel joints' names
            if (config["diff_cont"]["ros__parameters"]["left_wheel_names"] && config["diff_cont"]["ros__parameters"]["left_wheel_names"].IsSequence()) {
                YAML::Node left_wheel_names = config["diff_cont"]["ros__parameters"]["left_wheel_names"];
                for (std::size_t i = 0; i < left_wheel_names.size(); ++i) {
                    std::string joint_name = left_wheel_names[i].as<std::string>();
                    left_wheel_names[i] = add_prefix(prefix, joint_name);
                }
            }

            // Add the prefix to the right wheel joints' names
            if (config["diff_cont"]["ros__parameters"]["right_wheel_names"] && config["diff_cont"]["ros__parameters"]["right_wheel_names"].IsSequence()) {
                YAML::Node right_wheel_names = config["diff_cont"]["ros__parameters"]["right_wheel_names"];
                for (std::size_t i = 0; i < right_wheel_names.size(); ++i) {
                    std::string joint_name = right_wheel_names[i].as<std::string>();
                    right_wheel_names[i] = add_prefix(prefix, joint_name);
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure: missing keys 'diff_cont' or 'ros__parameters'");
            return;
        }

        // Write the modifications to a file
        std::ofstream fout(out_file);
        
        // Add a comment
        fout << "# Automatically generated file to prepend the prefix to frame and joint names.\n";
        fout << "# Do not modify this file directly. Instead, modify the source file: " << file_path << "\n\n";

        fout << config; // Add the new parameters to the file
        fout.close();
        RCLCPP_INFO(this->get_logger(), "YAML file successfully modified and saved as '%s'", out_file.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YamlModifierNode>();

    rclcpp::shutdown();
    return 0;
}
