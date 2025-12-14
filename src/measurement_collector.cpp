#include <rclcpp/rclcpp.hpp>

#include "ros2_lidar_georeference/msg/measurement_collect.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <fstream>

static constexpr const char* IDLE_UUID =
    "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx";

enum class JobType { POINTS, POSITION };

struct Job {
    JobType type;
    rclcpp::Time stamp;
    std::vector<uint8_t> payload;
};

class MeasurementRecorder : public rclcpp::Node
{
public:
    MeasurementRecorder()
        : Node("measurement_recorder")
    {
        declare_parameter<int>("skip", 1);
        declare_parameter<int>("workers", 4);

        get_parameter("skip", skip_);
        get_parameter("workers", worker_count_);

        if (skip_ < 1) skip_ = 1;
        if (worker_count_ < 1) worker_count_ = 1;

        auto base = std::filesystem::path(std::getenv("HOME")) / ".ros" / "ros2_lidar_georeference";
        points_dir_   = (base / "raw/points").string();
        position_dir_ = (base / "raw/position").string();
        std::filesystem::create_directories(points_dir_);
        std::filesystem::create_directories(position_dir_);

        collect_sub_ = create_subscription<ros2_lidar_georeference::msg::MeasurementCollect>(
            "/measurement/collect", 10,
            std::bind(&MeasurementRecorder::collectCallback, this, std::placeholders::_1));

        rclcpp::QoS lidar_qos(rclcpp::KeepLast(5));
        lidar_qos.best_effort();

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points",
            lidar_qos,
            std::bind(&MeasurementRecorder::velodyneCallback, this, std::placeholders::_1));

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        fixposition_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fixposition/odometry_ecef",
            qos,
            std::bind(&MeasurementRecorder::fixpositionCallback, this, std::placeholders::_1));

        startWorkers();

        RCLCPP_INFO(get_logger(),
            "Recorder ready (skip=%d, workers=%d)", skip_, worker_count_);
    }

    ~MeasurementRecorder()
    {
        stopWorkers();
    }

private:
    // ---------------- Config ----------------
    int skip_;
    int worker_count_;
    uint64_t velodyne_counter_ = 0;

    // ---------------- State ----------------
    std::atomic<bool> recording_{false};
    std::atomic<bool> shutdown_{false};

    // ---------------- Paths ----------------
    std::string pkg_root_;
    std::string points_dir_;
    std::string position_dir_;

    // ---------------- Worker infra ----------------
    std::queue<Job> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<std::thread> workers_;

    // ---------------- ROS ----------------
    rclcpp::Subscription<ros2_lidar_georeference::msg::MeasurementCollect>::SharedPtr collect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fixposition_sub_;

    // ---------------- Worker control ----------------
    void startWorkers()
    {
        shutdown_ = false;
        for (int i = 0; i < worker_count_; ++i)
            workers_.emplace_back(&MeasurementRecorder::workerLoop, this);
    }

    void stopWorkers()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
            std::queue<Job> empty;
            std::swap(queue_, empty);   // DROP remaining jobs immediately
        }
        cv_.notify_all();

        for (auto &t : workers_)
            if (t.joinable()) t.join();

        workers_.clear();
    }

    // ---------------- Callbacks ----------------
    void collectCallback(const ros2_lidar_georeference::msg::MeasurementCollect::SharedPtr msg)
    {
        if (msg->uuid == IDLE_UUID)
        {
            recording_ = false;
            velodyne_counter_ = 0;

            stopWorkers();
            startWorkers();   // fresh workers for next session

            RCLCPP_INFO(get_logger(), "Recording stopped (workers killed)");
            return;
        }

        recording_ = true;
        velodyne_counter_ = 0;

        RCLCPP_INFO(get_logger(), "Recording started");
    }

    void velodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!recording_)
            return;

        velodyne_counter_++;
        if ((velodyne_counter_ - 1) % skip_ != 0)
            return;

        std::vector<uint8_t> buffer;
        buffer.reserve(msg->width * msg->height * 12);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            float xyz[3] = {*it_x, *it_y, *it_z};
            buffer.insert(buffer.end(),
                reinterpret_cast<uint8_t*>(xyz),
                reinterpret_cast<uint8_t*>(xyz) + sizeof(xyz));
        }

        enqueue(Job{
            JobType::POINTS,
            msg->header.stamp,
            std::move(buffer)
        });
    }

    void fixpositionCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!recording_)
            return;

        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // ECEF position (meters) + orientation (radians)
        double data[6] = {
            p.x, p.y, p.z,   // ECEF XYZ
            roll, pitch, yaw
        };

        std::vector<uint8_t> buffer(
            reinterpret_cast<uint8_t*>(data),
            reinterpret_cast<uint8_t*>(data) + sizeof(data));

        enqueue(Job{
            JobType::POSITION,
            msg->header.stamp,
            std::move(buffer)
        });
    }

    // ---------------- Queue ops ----------------
    void enqueue(Job &&job)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) return;
            queue_.push(std::move(job));
        }
        cv_.notify_one();
    }

    // ---------------- Worker loop ----------------
    void workerLoop()
    {
        while (true)
        {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] {
                    return shutdown_ || !queue_.empty();
                });

                if (shutdown_)
                    return;

                job = std::move(queue_.front());
                queue_.pop();
            }

            writeJob(job);
        }
    }

    // ---------------- Disk I/O ----------------
    void writeJob(const Job &job)
    {
        int64_t ns = job.stamp.nanoseconds();
        int64_t sec  = ns / 1000000000LL;
        int64_t nsec = ns % 1000000000LL;

        std::stringstream path;

        if (job.type == JobType::POINTS)
            path << points_dir_;
        else
            path << position_dir_;

        path << "/" << sec << "_" << nsec << ".bin";

        std::ofstream out(path.str(), std::ios::binary);
        if (!out) return;

        out.write(reinterpret_cast<const char*>(job.payload.data()),
                job.payload.size());
        out.close();
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementRecorder>());
    rclcpp::shutdown();
    return 0;
}