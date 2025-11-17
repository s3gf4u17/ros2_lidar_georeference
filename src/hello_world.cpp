#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>

namespace fs = std::filesystem;

class MeasurementPublisher : public rclcpp::Node
{
public:
    MeasurementPublisher()
    : Node("measurement_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/thesis/measurements", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                         std::bind(&MeasurementPublisher::publishMeasurements, this));

        RCLCPP_INFO(this->get_logger(), "MeasurementPublisher node started.");
    }

private:
    void publishMeasurements()
    {
        const char* base_env = "/home/pi/ros2_lidar_georeference_data";
        if (!base_env) {
            RCLCPP_ERROR(this->get_logger(), "Environment variable ROS2_LIDAR_GEOREFERENCE_DATA not set!");
            return;
        }

        fs::path config_dir = fs::path(base_env) / "config";

        if (!fs::exists(config_dir) || !fs::is_directory(config_dir)) {
            RCLCPP_ERROR(this->get_logger(), "Config directory not found: %s", config_dir.c_str());
            return;
        }

        std::vector<std::string> json_contents;

        for (const auto& entry : fs::directory_iterator(config_dir)) {
            if (entry.path().extension() == ".json") {
                std::ifstream file(entry.path());
                if (!file.is_open()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to open file: %s", entry.path().c_str());
                    continue;
                }

                std::stringstream buffer;
                buffer << file.rdbuf();
                json_contents.push_back(buffer.str());
                file.close();
            }
        }

        if (json_contents.empty()) {
            RCLCPP_WARN(this->get_logger(), "No JSON config files found in: %s", config_dir.c_str());
            return;
        }

        // Combine JSON strings into a simple JSON array
        std::ostringstream message_stream;
        message_stream << "[";
        for (size_t i = 0; i < json_contents.size(); ++i) {
            message_stream << json_contents[i];
            if (i != json_contents.size() - 1)
                message_stream << ",";
        }
        message_stream << "]";

        auto msg = std_msgs::msg::String();
        msg.data = message_stream.str();

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu measurement configs.", json_contents.size());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementPublisher>());
    rclcpp::shutdown();
    return 0;
}