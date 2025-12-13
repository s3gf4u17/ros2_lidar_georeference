#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "ros2_lidar_georeference/msg/file_list.hpp"
#include "ros2_lidar_georeference/srv/file_delete.hpp"

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

static const std::string CONFIG_DIR =
    "/var/www/ros2_lidar_georeference/downloads/";

class ConfigManagerNode : public rclcpp::Node
{
public:
  ConfigManagerNode()
  : Node("config_manager_node")
  {
    publisher_ = this->create_publisher<ros2_lidar_georeference::msg::FileList>(
        "/thesis/configs", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ConfigManagerNode::publish_configs, this));

    service_ = this->create_service<ros2_lidar_georeference::srv::FileDelete>(
        "/thesis/delete_file",
        std::bind(
            &ConfigManagerNode::handle_delete_request,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Config Manager Node started");

    RCLCPP_INFO(
  this->get_logger(),
  "Running as UID=%d GID=%d",
  getuid(),
  getgid()
);
  }

private:
  void publish_configs()
  {
    json config_list = json::array();

    try
    {
      for (const auto & entry : fs::directory_iterator(CONFIG_DIR))
      {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".cfg")
        {
          std::ifstream file(entry.path());
          if (!file.is_open())
          {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to open %s",
                entry.path().c_str());
            continue;
          }

          json cfg;
          try
          {
            file >> cfg;
            config_list.push_back(cfg);
          }
          catch (const json::parse_error & e)
          {
            RCLCPP_ERROR(
                this->get_logger(),
                "JSON parse error in %s: %s",
                entry.path().c_str(),
                e.what());
          }
        }
      }
    }
    catch (const fs::filesystem_error & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", e.what());
    }

    ros2_lidar_georeference::msg::FileList msg;
    msg.json_list = config_list.dump();

    publisher_->publish(msg);
  }

  void handle_delete_request(
      const std::shared_ptr<ros2_lidar_georeference::srv::FileDelete::Request> request,
      std::shared_ptr<ros2_lidar_georeference::srv::FileDelete::Response> response)
  {
    bool deleted_any = false;

    try
    {
      for (const auto & entry : fs::directory_iterator(CONFIG_DIR))
      {
        if (!entry.is_regular_file())
          continue;

        const std::string filename = entry.path().filename().string();

        if (filename.rfind(request->uuid, 0) == 0)
        {
          fs::remove(entry.path());
          deleted_any = true;

          RCLCPP_INFO(
              this->get_logger(),
              "Deleted file: %s",
              entry.path().c_str());
        }
      }
    }
    catch (const fs::filesystem_error & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", e.what());
      response->response_value = -1;
      return;
    }

    response->response_value = deleted_any ? 0 : -1;
  }

  rclcpp::Publisher<ros2_lidar_georeference::msg::FileList>::SharedPtr publisher_;
  rclcpp::Service<ros2_lidar_georeference::srv::FileDelete>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConfigManagerNode>());
  rclcpp::shutdown();
  return 0;
}