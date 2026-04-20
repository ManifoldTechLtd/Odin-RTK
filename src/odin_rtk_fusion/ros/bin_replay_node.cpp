/*
 * bin_replay_node: ROS1 node that reads ODINBIN file and replays through fusion.
 *
 * Publishes:
 *   /world_enu/odom          (nav_msgs/Odometry)   - fused odometry
 *   /world_enu/gnss_marker   (visualization_msgs/Marker) - GNSS position marker
 *   /tf                      (world->odom transform)
 *
 * Parameters:
 *   ~bin_file     (string)  - path to .bin file
 *   ~config_file  (string)  - path to fusion_params.yaml
 *   ~playback_speed (double) - playback speed multiplier (default 1.0, 0=no wait)
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <eigen3/Eigen/Dense>

#ifdef ODIN_ROS2
  #error "bin_replay_node is ROS1 only"
#else
  #include <ros/ros.h>
  #include <nav_msgs/Odometry.h>
  #include <geometry_msgs/TransformStamped.h>
  #include <visualization_msgs/Marker.h>
  #include <tf2_ros/transform_broadcaster.h>
#endif

#include "common/build_config.h"
#include "common/data_types.hpp"
#include "common/parameter.hpp"
#include "common/enu_converter.hpp"
#include "common/logger.hpp"
#include "common/proj_converter.hpp"
#include "fusion/fusion_base.hpp"
#include "fusion/fusion_factory.hpp"

namespace odin_rtk_fusion {

using OdometryMsg = nav_msgs::Odometry;
using MarkerMsg = visualization_msgs::Marker;
using TransformStampedMsg = geometry_msgs::TransformStamped;

// ============================================================================
// Binary reader structs (packed to match file format exactly)
// ============================================================================
#pragma pack(push, 1)
struct BinHeader {
    char magic[8];
    uint32_t version;
    uint64_t num_records;
};

struct OdomPayload {
    double px, py, pz;
    double qw, qx, qy, qz;
};

struct BestposPayload {
    double lat, lon, hgt;
    float undulation;
    float lat_stdev, lon_stdev, hgt_stdev;
    uint32_t sol_status, pos_type;
    uint8_t num_sol_svs;
    float diff_age, sol_age;
};

struct Heading2Payload {
    float heading, pitch;
    float heading_stdev, pitch_stdev;
    float length;
    uint32_t sol_status, pos_type;
};
#pragma pack(pop)

// ============================================================================
// pos_type (NovAtel BESTPOS) to GGA quality
// ============================================================================
static int posTypeToGgaQuality(uint32_t pos_type) {
    switch (pos_type) {
        case 48: case 49: case 50: case 56: return 4;  // RTK Fix
        case 32: case 33: case 34: case 55: return 5;  // RTK Float
        case 17: case 18: case 54: case 52: return 2;  // DGPS
        case 16: case 53: return 1;                      // Single
        case 68: case 69: case 73: case 74:
        case 77: case 78: case 79: case 80: return 2;  // PPP
        default: return 0;
    }
}

// ============================================================================
// BinReplayNode
// ============================================================================
class BinReplayNode {
public:
    BinReplayNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // Parameters
        pnh_.param<std::string>("bin_file", bin_file_, "");
        pnh_.param<std::string>("config_file", config_file_, "");
        pnh_.param<double>("playback_speed", playback_speed_, 1.0);

        if (bin_file_.empty()) {
            ROS_FATAL("bin_file parameter is required!");
            ros::shutdown();
            return;
        }

        // Load fusion config
        FusionAlgorithmParams new_config;
        std::string algorithm_type = "graph";
        std::string input_lla_crs = "EPSG:4326";
        try {
            const FusionNodeParams params = FusionNodeParams::LoadFromYamlFile(config_file_);
            new_config = params.algorithm;
            algorithm_type = params.algorithm_type;
            input_lla_crs = params.input_lla_crs;
            output_project_enable_ = params.output_project_enable;
            output_project_target_crs_ = params.output_project_target_crs;
        } catch (const std::exception& e) {
            ROS_WARN("Failed to load config '%s': %s, using defaults", config_file_.c_str(), e.what());
        }

        // Allow ROS parameter override (takes precedence over YAML).
        // Empty string means "keep the YAML value".
        std::string algo_override;
        pnh_.param<std::string>("algorithm_type", algo_override, std::string(""));
        if (!algo_override.empty()) {
            algorithm_type = algo_override;
        }

        if (!enu_converter_.init(input_lla_crs)) {
            ROS_WARN("ENU converter init failed, fallback to EPSG:4326");
            enu_converter_.init("EPSG:4326");
        }

        if (output_project_enable_ && !output_project_target_crs_.empty()) {
            if (!proj_converter_.init(input_lla_crs, output_project_target_crs_)) {
                ROS_WARN("PROJ converter init failed, disabling projection");
                output_project_enable_ = false;
            }
        }

        // Publishers
        fused_odom_pub_ = nh_.advertise<OdometryMsg>("/world_enu/odom", 100);
        gnss_marker_pub_ = nh_.advertise<MarkerMsg>("/world_enu/gnss_marker", 10);
        projected_odom_pub_ = nh_.advertise<OdometryMsg>("/world/odom_projected", 100);

        // Raw odom replay publishers
        odom_highfreq_pub_ = nh_.advertise<OdometryMsg>("/odin1/odometry_highfreq", 200);
        h1_odom_pub_ = nh_.advertise<OdometryMsg>("/h1_odom", 50);

        // Fusion
        fusion_ = createFusion(algorithm_type, new_config);

        ROS_INFO("BinReplayNode: bin_file=%s speed=%.1f algorithm=%s",
                 bin_file_.c_str(), playback_speed_, algorithm_type.c_str());
    }

    void run() {
        // Open binary file
        std::ifstream in(bin_file_, std::ios::binary);
        if (!in) {
            ROS_FATAL("Failed to open bin file: %s", bin_file_.c_str());
            return;
        }

        BinHeader header;
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (std::strncmp(header.magic, "ODINBIN", 7) != 0) {
            ROS_FATAL("Invalid bin file magic");
            return;
        }

        ROS_INFO("ODINBIN v%u: %lu records", header.version, header.num_records);

        double first_ts = 0.0;
        auto wall_start = std::chrono::steady_clock::now();
        uint64_t odom_count = 0, gps_count = 0, heading_count = 0;

        for (uint64_t i = 0; i < header.num_records && ros::ok(); i++) {
            uint8_t type;
            double timestamp;
            in.read(reinterpret_cast<char*>(&type), 1);
            in.read(reinterpret_cast<char*>(&timestamp), 8);

            // Read topic name
            uint16_t topic_len = 0;
            in.read(reinterpret_cast<char*>(&topic_len), 2);
            std::string topic_name(topic_len, '\0');
            if (topic_len > 0) {
                in.read(&topic_name[0], topic_len);
            }

            if (i == 0) first_ts = timestamp;

            // Playback timing
            if (playback_speed_ > 0.0) {
                const double elapsed_data = timestamp - first_ts;
                const double elapsed_wall =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
                const double target_wall = elapsed_data / playback_speed_;
                if (target_wall > elapsed_wall) {
                    const double sleep_sec = target_wall - elapsed_wall;
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(static_cast<int64_t>(sleep_sec * 1e6)));
                }
            }

            const ros::Time stamp(timestamp);

            if (type == 0) {  // ODOM (odometry_highfreq) -> feed to fusion + republish
                OdomPayload odom_p;
                in.read(reinterpret_cast<char*>(&odom_p), sizeof(odom_p));
                processOdom(odom_p, timestamp, stamp);
                publishRawOdom(odom_highfreq_pub_, odom_p, stamp, "odom");
                odom_count++;
            } else if (type == 3) {  // ODOM2 (h1_odom) -> republish only, not fed to fusion
                OdomPayload odom_p;
                in.read(reinterpret_cast<char*>(&odom_p), sizeof(odom_p));
                publishRawOdom(h1_odom_pub_, odom_p, stamp, "odom");
            } else if (type == 1) {
                BestposPayload bp;
                in.read(reinterpret_cast<char*>(&bp), sizeof(bp));
                processBestpos(bp, timestamp, stamp);
                gps_count++;
            } else if (type == 2) {
                Heading2Payload h2;
                in.read(reinterpret_cast<char*>(&h2), sizeof(h2));
                heading_count++;
            } else {
                ROS_WARN_ONCE("Unknown record type %u topic=%s", type, topic_name.c_str());
            }

            // Process ROS callbacks (for rviz etc.)
            ros::spinOnce();

            if ((i % 10000) == 0) {
                ROS_INFO("Progress: %lu/%lu (odom=%lu gps=%lu)", i, header.num_records, odom_count, gps_count);
            }
        }

        ROS_INFO("Replay done! odom=%lu gps=%lu heading=%lu", odom_count, gps_count, heading_count);

        // Keep node alive for rviz inspection
        ROS_INFO("Replay finished. Press Ctrl+C to exit.");
        ros::spin();
    }

private:
    void processOdom(const OdomPayload& p, double timestamp, const ros::Time& stamp) {
        OdomData odom;
        odom.timestamp = timestamp;
        Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
        q.normalize();
        odom.pose = Eigen::Matrix4d::Identity();
        odom.pose.block<3,3>(0,0) = q.toRotationMatrix();
        odom.pose(0,3) = p.px;
        odom.pose(1,3) = p.py;
        odom.pose(2,3) = p.pz;

        fusion_->addOdom(odom, false);

        if (!fusion_->isInitialized()) return;

        const Eigen::Matrix4d world_T_odom = fusion_->getWorldToOdom();
        const Eigen::Matrix4d world_T_body = world_T_odom * odom.pose;

        // ---- Jump quantification ----
        const Eigen::Vector3d fused_pos(world_T_body(0,3), world_T_body(1,3), world_T_body(2,3));
        const Eigen::Vector3d odom_pos(p.px, p.py, p.pz);
        if (have_prev_fused_) {
            const double fused_delta = (fused_pos - prev_fused_pos_).norm();
            const double odom_delta  = (odom_pos  - prev_odom_pos_).norm();
            const double jitter = std::abs(fused_delta - odom_delta);
            jitter_sum_ += jitter;
            jitter_count_++;
            if (jitter > jitter_max_) jitter_max_ = jitter;
            if (jitter > 0.01) {  // >1cm
                jitter_events_++;
                LOG_WARN("[JITTER] {:.4f}m  fused_d={:.4f} odom_d={:.4f}  (max={:.4f} avg={:.4f} events={}/{})",
                         jitter, fused_delta, odom_delta,
                         jitter_max_, jitter_sum_ / jitter_count_, jitter_events_, jitter_count_);
            } else if ((jitter_count_ % 5000) == 0) {
                LOG_INFO("[JITTER-STATS] max={:.4f}m avg={:.6f}m events(>1cm)={}/{}",
                         jitter_max_, jitter_sum_ / jitter_count_, jitter_events_, jitter_count_);
            }
        }
        prev_fused_pos_ = fused_pos;
        prev_odom_pos_  = odom_pos;
        have_prev_fused_ = true;

        // Publish fused odom
        OdometryMsg msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = world_T_body(0,3);
        msg.pose.pose.position.y = world_T_body(1,3);
        msg.pose.pose.position.z = world_T_body(2,3);
        Eigen::Quaterniond q_out(world_T_body.block<3,3>(0,0));
        q_out.normalize();
        msg.pose.pose.orientation.w = q_out.w();
        msg.pose.pose.orientation.x = q_out.x();
        msg.pose.pose.orientation.y = q_out.y();
        msg.pose.pose.orientation.z = q_out.z();
        fused_odom_pub_.publish(msg);

        // Publish projected odom
        publishProjectedOdom(msg);

        // Publish TF
        publishTF(stamp);
    }

    void processBestpos(const BestposPayload& bp, double timestamp, const ros::Time& stamp) {
        if (bp.sol_status != 0) return;

        const int quality = posTypeToGgaQuality(bp.pos_type);
        if (quality == 0) return;

        const double alt = bp.hgt + static_cast<double>(bp.undulation);
        const double hdop = std::sqrt(
            static_cast<double>(bp.lat_stdev) * bp.lat_stdev +
            static_cast<double>(bp.lon_stdev) * bp.lon_stdev);

        if (!enu_converter_.isInitialized()) {
            enu_converter_.setOrigin(bp.lat, bp.lon, alt);
            ROS_INFO("ENU origin: lat=%.7f lon=%.7f alt=%.2f", bp.lat, bp.lon, alt);
        }

        Eigen::Vector3d enu = enu_converter_.toENU(bp.lat, bp.lon, alt);

        // Publish GNSS marker
        MarkerMsg m;
        m.header.stamp = stamp;
        m.header.frame_id = "world";
        m.ns = "gnss";
        m.id = 0;
        m.type = MarkerMsg::SPHERE;
        m.action = MarkerMsg::ADD;
        m.pose.orientation.w = 1.0;
        m.pose.position.x = enu.x();
        m.pose.position.y = enu.y();
        m.pose.position.z = enu.z();
        if (quality == 4) { m.color.r=0; m.color.g=1; m.color.b=0; m.color.a=1; }
        else if (quality == 5) { m.color.r=0; m.color.g=0; m.color.b=1; m.color.a=1; }
        else { m.color.r=1; m.color.g=1; m.color.b=0; m.color.a=1; }
        double s = (quality >= 4) ? std::min(std::max(0.3, 0.15*hdop), 2.0) : 2.0;
        m.scale.x = s; m.scale.y = s; m.scale.z = s;
        gnss_marker_pub_.publish(m);

        // Feed fusion
        GPSData gps;
        gps.timestamp = timestamp;
        gps.lat = bp.lat;
        gps.lon = bp.lon;
        gps.alt = alt;
        gps.quality = quality;
        gps.hdop = hdop;
        gps.num_satellites = static_cast<int>(bp.num_sol_svs);
        gps.valid = true;
        gps.enu = enu;
        fusion_->addGPS(gps);
    }

    void publishProjectedOdom(const OdometryMsg& enu_msg) {
        if (!output_project_enable_ || !proj_converter_.isInitialized() || !enu_converter_.isInitialized())
            return;

        Eigen::Vector3d enu(enu_msg.pose.pose.position.x,
                            enu_msg.pose.pose.position.y,
                            enu_msg.pose.pose.position.z);
        double lat, lon, alt;
        enu_converter_.toLLA(enu, lat, lon, alt);
        double ox, oy, oz;
        if (!proj_converter_.projectDeg(lat, lon, alt, ox, oy, oz)) return;

        OdometryMsg proj = enu_msg;
        proj.header.frame_id = "world_proj";
        proj.pose.pose.position.x = enu_msg.pose.pose.position.x;
        proj.pose.pose.position.y = enu_msg.pose.pose.position.y;
        proj.pose.pose.position.z = enu_msg.pose.pose.position.z;
        projected_odom_pub_.publish(proj);
    }

    void publishTF(const ros::Time& stamp) {
        Eigen::Matrix4d wTo = fusion_->getWorldToOdom();
        TransformStampedMsg tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = "odom";
        tf_msg.transform.translation.x = wTo(0,3);
        tf_msg.transform.translation.y = wTo(1,3);
        tf_msg.transform.translation.z = wTo(2,3);
        Eigen::Quaterniond q(wTo.block<3,3>(0,0));
        q.normalize();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_broadcaster_.sendTransform(tf_msg);
    }

    void publishRawOdom(ros::Publisher& pub, const OdomPayload& p,
                        const ros::Time& stamp, const std::string& frame_id) {
        OdometryMsg msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = p.px;
        msg.pose.pose.position.y = p.py;
        msg.pose.pose.position.z = p.pz;
        msg.pose.pose.orientation.w = p.qw;
        msg.pose.pose.orientation.x = p.qx;
        msg.pose.pose.orientation.y = p.qy;
        msg.pose.pose.orientation.z = p.qz;
        pub.publish(msg);
    }

    // Members
    ros::NodeHandle nh_, pnh_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<FusionBase> fusion_;
    ENUConverter enu_converter_;
    ProjConverter proj_converter_;

    ros::Publisher fused_odom_pub_;
    ros::Publisher gnss_marker_pub_;
    ros::Publisher projected_odom_pub_;
    ros::Publisher odom_highfreq_pub_;
    ros::Publisher h1_odom_pub_;

    std::string bin_file_;
    std::string config_file_;
    double playback_speed_ = 1.0;
    bool output_project_enable_ = false;
    std::string output_project_target_crs_;

    // Jump quantification
    bool have_prev_fused_ = false;
    Eigen::Vector3d prev_fused_pos_{0,0,0};
    Eigen::Vector3d prev_odom_pos_{0,0,0};
    double jitter_max_ = 0.0;
    double jitter_sum_ = 0.0;
    uint64_t jitter_count_ = 0;
    uint64_t jitter_events_ = 0;
};

}  // namespace odin_rtk_fusion

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_replay_node");
    INIT_SPDLOG_CLEAR((WORK_SPACE_PATH + "/log/log.txt").c_str());
    SET_LOG_LEVEL(spdlog::level::debug);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    odin_rtk_fusion::BinReplayNode node(nh, pnh);
    node.run();

    return 0;
}
