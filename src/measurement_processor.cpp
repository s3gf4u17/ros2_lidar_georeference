#include <rclcpp/rclcpp.hpp>
#include "ros2_lidar_georeference/srv/measurement_process.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <filesystem>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

using MeasurementProcess =
    ros2_lidar_georeference::srv::MeasurementProcess;

namespace fs = std::filesystem;

static constexpr const char* RAW_ROOT =
    ".ros/ros2_lidar_georeference/raw";
static constexpr const char* OUTPUT_ROOT =
    "/var/www/ros2_lidar_georeference/downloads";

/* ---------------- LAS structs ---------------- */

struct LasHeader {
    uint8_t FileSignatureLASF[4];
    uint16_t fileSourceID, globalEncoding;
    uint32_t guidData1;
    uint16_t guidData2, guidData3;
    uint8_t guidData4[8], versionMaj, versionMin;
    uint8_t systemIdentifier[32], generatingSoftware[32];
    uint16_t genDay, genYear, headerSize;
    uint32_t offsetToPointData, numOfVarLenRecords;
    uint8_t pointDataFormat;
    uint16_t pointDataRecordLen;
    uint32_t numOfPointRecords, numOfPointByReturn[5];
    double x_sca, y_sca, z_sca, x_off, y_off, z_off;
    double x_max, x_min, y_max, y_min, z_max, z_min;
} __attribute__((packed));

struct LasPointRecord {
    int32_t xpos, ypos, zpos;
    uint16_t intensity;
    uint8_t flags, classification, scanAngleRank, userData;
    uint16_t pointSourceID;
    double gps_time;
} __attribute__((packed));

/* ---------------- Pose container ---------------- */

struct PoseECEF {
    double x, y, z;
    double roll, pitch, yaw;
};

void cleanup_raw_files(const fs::path& points_dir,
                       const fs::path& pos_dir)
{
    auto cleanup_dir = [&](const fs::path& dir)
    {
        if (!fs::exists(dir) || !fs::is_directory(dir))
            return;

        for (const auto& entry : fs::directory_iterator(dir))
        {
            if (entry.is_regular_file())
            {
                std::error_code ec;
                fs::remove(entry.path(), ec);
                if (ec)
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("measurement_processor"),
                        "Failed to remove file %s: %s",
                        entry.path().c_str(),
                        ec.message().c_str());
                }
            }
        }
    };

    cleanup_dir(points_dir);
    cleanup_dir(pos_dir);
}

/* ---------------- Node ---------------- */

class MeasurementProcessor : public rclcpp::Node
{
public:
    MeasurementProcessor()
        : Node("measurement_processor")
    {
        srv_ = create_service<MeasurementProcess>(
            "/measurement/process",
            std::bind(&MeasurementProcessor::handle, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Measurement processor ready");
    }

private:
    rclcpp::Service<MeasurementProcess>::SharedPtr srv_;

    /* ---------- Utilities ---------- */

    static int64_t parseStamp(const fs::path& p)
    {
        auto s = p.stem().string();
        auto pos = s.find('_');
        int64_t sec = std::stoll(s.substr(0, pos));
        int64_t nsec = std::stoll(s.substr(pos + 1));
        return sec * 1000000000LL + nsec;
    }

    static PoseECEF readPose(const fs::path& p)
    {
        PoseECEF pose{};
        std::ifstream in(p, std::ios::binary);
        in.read(reinterpret_cast<char*>(&pose), sizeof(PoseECEF));
        return pose;
    }

    /* ---------- Core processing ---------- */

    void handle(
        const std::shared_ptr<MeasurementProcess::Request> req,
        std::shared_ptr<MeasurementProcess::Response> res)
    {
        try {
            process(req->uuid);
            res->response_value = 0;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Measurement processing failed for uuid '%s': %s",
                req->uuid.c_str(),
                e.what()
            );
            res->response_value = -2;
        }
        catch (...) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Measurement processing failed for uuid '%s': unknown exception",
                req->uuid.c_str()
            );
            res->response_value = -2;
        }
    }

    void process(const std::string& uuid)
    {
        fs::path raw = fs::path("/var/tmp/ros2_lidar_georeference") / "raw";
        fs::path points_dir = raw / "points";
        fs::path pos_dir = raw / "position";

        RCLCPP_INFO(get_logger(), "Measurement processor launched");

        RCLCPP_INFO(
            this->get_logger(),
            "Looking for raw data:\n  points_dir = %s\n  position_dir = %s",
            points_dir.string().c_str(),
            pos_dir.string().c_str()
        );

        if (!fs::exists(points_dir) || !fs::exists(pos_dir))
            throw std::runtime_error("Missing raw data");

        RCLCPP_INFO(get_logger(), "RAW data exists");

        /* ---- Load poses ---- */

        std::map<int64_t, PoseECEF> poses;

        for (auto& f : fs::directory_iterator(pos_dir)) {
            poses[parseStamp(f.path())] = readPose(f.path());
        }

        /* ---- Open LAS ---- */

        fs::path out = fs::path(OUTPUT_ROOT) / (uuid + ".las");
        std::ofstream las(out, std::ios::binary);

        LasHeader hdr{};
        memcpy(hdr.FileSignatureLASF, "LASF", 4);
        hdr.versionMaj = 1;
        hdr.versionMin = 2;
        hdr.headerSize = sizeof(LasHeader);
        hdr.offsetToPointData = sizeof(LasHeader);
        hdr.pointDataFormat = 1;
        hdr.pointDataRecordLen = sizeof(LasPointRecord);
        hdr.x_sca = hdr.y_sca = hdr.z_sca = 0.001;

        las.write(reinterpret_cast<char*>(&hdr), sizeof(hdr));

        uint32_t point_count = 0;

        /* ---- Process clouds ---- */

        for (auto& f : fs::directory_iterator(points_dir)) {

            int64_t t = parseStamp(f.path());

            auto it = poses.lower_bound(t);
            if (it == poses.end())
                continue;

            PoseECEF pose = it->second;

            tf2::Quaternion q;
            q.setRPY(pose.roll, pose.pitch, pose.yaw);
            tf2::Matrix3x3 R(q);

            std::ifstream in(f.path(), std::ios::binary);
            float xyz[3];

            while (in.read(reinterpret_cast<char*>(xyz), sizeof(xyz))) {

                tf2::Vector3 p_l(xyz[0], xyz[1], xyz[2]);
                tf2::Vector3 p_e = R * p_l +
                    tf2::Vector3(pose.x, pose.y, pose.z);

                LasPointRecord rec{};
                rec.xpos = static_cast<int32_t>(p_e.x() / hdr.x_sca);
                rec.ypos = static_cast<int32_t>(p_e.y() / hdr.y_sca);
                rec.zpos = static_cast<int32_t>(p_e.z() / hdr.z_sca);
                rec.gps_time = double(t) * 1e-9;

                las.write(reinterpret_cast<char*>(&rec), sizeof(rec));
                point_count++;
            }
        }

        /* ---- Finalize header ---- */

        hdr.numOfPointRecords = point_count;
        las.seekp(0);
        las.write(reinterpret_cast<char*>(&hdr), sizeof(hdr));
        las.close();

        /* ---- Cleanup ---- */

        cleanup_raw_files(points_dir, pos_dir);
    }
};

/* ---------------- main ---------------- */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementProcessor>());
    rclcpp::shutdown();
    return 0;
}
