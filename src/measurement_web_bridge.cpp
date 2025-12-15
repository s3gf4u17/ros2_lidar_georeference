#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/int32.hpp>

#include "ros2_lidar_georeference/action/measurement.hpp"

using namespace std::chrono_literals;

/* =======================
 *  Web → Action Bridge
 * ======================= */

class MeasurementWebBridge : public rclcpp::Node
{
public:
    using Measurement = ros2_lidar_georeference::action::Measurement;
    using GoalHandleMeasurement =
        rclcpp_action::ClientGoalHandle<Measurement>;

    MeasurementWebBridge()
        : Node("measurement_web_bridge")
    {
        /* ---------- Action client ---------- */
        action_client_ =
            rclcpp_action::create_client<Measurement>(
                this, "/measurement");

        /* ---------- Web subscriptions ---------- */
        web_goal_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/measurement/web/goal", 10,
            std::bind(&MeasurementWebBridge::onWebGoal, this,
                      std::placeholders::_1));

        web_result_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/measurement/web/result", 10,
            std::bind(&MeasurementWebBridge::onWebResult, this,
                      std::placeholders::_1));

        /* ---------- Web feedback publisher ---------- */
        web_feedback_pub_ =
            create_publisher<std_msgs::msg::Int32>(
                "/measurement/web/feedback", 10);

        /* ---------- Publish initial IDLE ---------- */
        publishFeedback(0);

        RCLCPP_INFO(get_logger(),
            "Measurement web bridge ready");
    }

private:
    /* ---------- ROS ---------- */

    rclcpp_action::Client<Measurement>::SharedPtr action_client_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr web_goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr web_result_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr web_feedback_pub_;

    GoalHandleMeasurement::SharedPtr goal_handle_;

    /* =======================
     *  Helpers
     * ======================= */

    void publishFeedback(int state)
    {
        std_msgs::msg::Int32 msg;
        msg.data = state;
        web_feedback_pub_->publish(msg);
    }

    /* =======================
     *  Web callbacks
     * ======================= */

    /* ---- START MEASUREMENT ---- */
    void onWebGoal(const std_msgs::msg::Int32 &)
    {
        if (goal_handle_) {
            RCLCPP_WARN(get_logger(),
                "Measurement already active");
            return;
        }

        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(),
                "Measurement action server not available");
            return;
        }

        Measurement::Goal goal;
        goal.request_value = 1;

        rclcpp_action::Client<Measurement>::SendGoalOptions options;

        /* ---- Goal response ---- */
        options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleMeasurement> handle)
            {
                if (!handle) {
                    RCLCPP_ERROR(get_logger(),
                        "Goal rejected by action server");
                    publishFeedback(0);
                    return;
                }

                goal_handle_ = handle;
                RCLCPP_INFO(get_logger(),
                    "Goal accepted");
            };

        /* ---- Feedback ---- */
        options.feedback_callback =
            [this](GoalHandleMeasurement::SharedPtr,
                   const std::shared_ptr<
                       const Measurement::Feedback> feedback)
            {
                publishFeedback(feedback->state);
            };

        /* ---- Result ---- */
        options.result_callback =
            [this](const GoalHandleMeasurement::WrappedResult & result)
            {
                publishFeedback(0);

                if (result.code ==
                    rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(get_logger(),
                        "Measurement finished successfully");
                }
                else {
                    RCLCPP_WARN(get_logger(),
                        "Measurement failed or canceled");
                }

                goal_handle_.reset();
            };

        action_client_->async_send_goal(goal, options);

        RCLCPP_INFO(get_logger(),
            "Sent measurement goal");
    }

    /* ---- STOP / CANCEL ---- */
    void onWebResult(const std_msgs::msg::Int32 &)
    {
        if (!goal_handle_) {
            RCLCPP_WARN(get_logger(),
                "No active measurement to cancel");
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Canceling measurement");

        action_client_->async_cancel_goal(goal_handle_);
    }
};

/* =======================
 *  main
 * ======================= */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementWebBridge>());
    rclcpp::shutdown();
    return 0;
}
