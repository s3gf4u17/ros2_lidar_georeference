#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ros2_lidar_georeference/action/measurement.hpp"
#include "ros2_lidar_georeference/msg/measurement_collect.hpp"
#include "ros2_lidar_georeference/srv/measurement_process.hpp"

#include <filesystem>
#include <fstream>
#include <random>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ctime>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

/* ---------------- Constants ---------------- */

static constexpr const char* IDLE_UUID =
    "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx";

static constexpr const char* DOWNLOAD_DIR =
    "/var/www/ros2_lidar_georeference/downloads";

/* ---------------- State ---------------- */

enum class State {
    IDLE = 0,
    COLLECTING = 1,
    PROCESSING = 2
};

/* ---------------- Node ---------------- */

class MeasurementManager : public rclcpp::Node
{
public:
    using Measurement = ros2_lidar_georeference::action::Measurement;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<Measurement>;

    MeasurementManager()
        : Node("measurement_manager")
    {
        collect_pub_ = create_publisher<
            ros2_lidar_georeference::msg::MeasurementCollect>(
            "/measurement/collect", 10);

        process_client_ = create_client<
            ros2_lidar_georeference::srv::MeasurementProcess>(
            "/measurement/process");

        action_server_ = rclcpp_action::create_server<Measurement>(
            this,
            "/measurement",
            std::bind(&MeasurementManager::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&MeasurementManager::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&MeasurementManager::handle_accepted, this,
                      std::placeholders::_1));

        fs::create_directories(DOWNLOAD_DIR);

        RCLCPP_INFO(get_logger(), "Measurement action server ready");
    }

private:
    /* -------- ROS -------- */

    rclcpp::Publisher<
        ros2_lidar_georeference::msg::MeasurementCollect>::SharedPtr collect_pub_;

    rclcpp::Client<
        ros2_lidar_georeference::srv::MeasurementProcess>::SharedPtr process_client_;

    rclcpp_action::Server<Measurement>::SharedPtr action_server_;

    /* -------- State -------- */

    std::mutex mutex_;
    State state_{State::IDLE};
    std::string active_uuid_;

    /* -------- Utilities -------- */

    static std::string generateUUID()
    {
        static std::mt19937_64 rng(std::random_device{}());
        static std::uniform_int_distribution<uint64_t> dist;

        uint64_t a = dist(rng);
        uint64_t b = dist(rng);

        std::stringstream ss;
        ss << std::hex << std::setfill('0')
           << std::setw(8) << (a >> 32) << "-"
           << std::setw(4) << ((a >> 16) & 0xFFFF) << "-"
           << std::setw(4) << (a & 0xFFFF) << "-"
           << std::setw(4) << (b >> 48) << "-"
           << std::setw(12) << (b & 0xFFFFFFFFFFFFULL);
        return ss.str();
    }

    static std::string nowISO()
    {
        auto t = std::time(nullptr);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%FT%TZ", std::gmtime(&t));
        return buf;
    }

    void writeCfg(const std::string& uuid,
                  const std::string& timestamp = "",
                  const std::string& download = "")
    {
        RCLCPP_INFO(get_logger(), "Writing .cfg file for UUID: %s", uuid.c_str());
        std::ofstream cfg(fs::path(DOWNLOAD_DIR) / (uuid + ".cfg"));

        cfg << "{\n";
        cfg << "  \"uuid\": \"" << uuid << "\",\n";

        if (timestamp.empty())
            cfg << "  \"timestamp\": null,\n";
        else
            cfg << "  \"timestamp\": \"" << timestamp << "\",\n";

        if (download.empty())
            cfg << "  \"download\": null\n";
        else
            cfg << "  \"download\": \"" << download << "\"\n";

        cfg << "}\n";
        cfg.close();
    }

    /* -------- Action callbacks -------- */

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Measurement::Goal>)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != State::IDLE) {
            RCLCPP_WARN(get_logger(), "Goal rejected - already active");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(get_logger(), "Cancel request accepted");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{
            std::bind(&MeasurementManager::execute, this, goal_handle)
        }.detach();
    }

    /* -------- Execution -------- */

    void execute(std::shared_ptr<GoalHandle> goal_handle)
    {
        auto feedback = std::make_shared<Measurement::Feedback>();
        auto result   = std::make_shared<Measurement::Result>();

        std::string uuid;
        
        // Generate UUID and create initial config file
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::COLLECTING;
            active_uuid_ = generateUUID();
            uuid = active_uuid_;
        }

        RCLCPP_INFO(get_logger(), "Generated UUID: %s", uuid.c_str());
        writeCfg(uuid);

        // Start collection - send UUID on /measurement/collect topic
        ros2_lidar_georeference::msg::MeasurementCollect start_msg;
        start_msg.uuid = uuid;
        collect_pub_->publish(start_msg);

        RCLCPP_INFO(get_logger(), "Started collection for UUID: %s", uuid.c_str());

        // Send feedback = 1 (COLLECTING)
        feedback->state = 1;
        goal_handle->publish_feedback(feedback);

        // Poll for result requests while in COLLECTING state
        // In this implementation, we wait for the action to be explicitly 
        // requested to move to next phase via the result/cancel mechanism
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_ != State::COLLECTING) {
                    break;
                }
            }

            // Check if goal is being canceled (this serves as the trigger to stop collecting)
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(get_logger(), "Stopping collection due to cancel request");
                break;
            }

            // Continue sending feedback = 1
            feedback->state = 1;
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        // Stop collection - send special UUID
        ros2_lidar_georeference::msg::MeasurementCollect stop_msg;
        stop_msg.uuid = IDLE_UUID;
        collect_pub_->publish(stop_msg);

        RCLCPP_INFO(get_logger(), "Stopped collection, starting processing");

        // Transition to PROCESSING state
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::PROCESSING;
        }

        // Send feedback = 2 (PROCESSING)
        feedback->state = 2;
        goal_handle->publish_feedback(feedback);

        // Call the MeasurementProcess service
        if (!process_client_->wait_for_service(10s)) {
            RCLCPP_ERROR(get_logger(), "MeasurementProcess service not available");
            result->success = false;
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = State::IDLE;
                active_uuid_.clear();
            }
            
            goal_handle->abort(result);
            return;
        }

        auto req = std::make_shared<
            ros2_lidar_georeference::srv::MeasurementProcess::Request>();
        req->uuid = uuid;

        RCLCPP_INFO(get_logger(), "Calling MeasurementProcess service");
        auto future = process_client_->async_send_request(req);
        
        // Wait for service response (this thread is safe to block)
        auto resp = future.get();
        RCLCPP_INFO(get_logger(), "MeasurementProcess returned: %d", 
                    resp->response_value);

        // Update config file with timestamp and download info
        std::string timestamp = nowISO();
        std::string download = std::string("/downloads/") + uuid + ".las";
        writeCfg(uuid, timestamp, download);

        // Transition back to IDLE
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::IDLE;
            active_uuid_.clear();
        }

        // Send feedback = 0 (IDLE)
        feedback->state = 0;
        goal_handle->publish_feedback(feedback);

        // Return result = 0 (success)
        result->success  = true;
        result->uuid     = uuid;
        result->download = download;

        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Measurement completed successfully");
    }
};

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementManager>());
    rclcpp::shutdown();
    return 0;
}