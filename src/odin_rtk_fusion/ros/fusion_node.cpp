/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Common includes
#include <cmath>
#include <string>

#include <eigen3/Eigen/Dense>

#include "common/build_config.h"
#include "common/data_types.hpp"
#include "common/parameter.h"
#include "common/enu_converter.hpp"
#include "common/gga_parser.hpp"
#include "common/logger.hpp"
#include "common/proj_converter.hpp"
#include "gtsam/fusion_base.hpp"
#include "gtsam/new_fusion.hpp"

// ROS-specific includes
#if defined(ODIN_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#else
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace odin_rtk_fusion {

// Type aliases for ROS1/ROS2 abstraction
#if defined(ODIN_ROS2)
using OdometryMsg = nav_msgs::msg::Odometry;
using SentenceMsg = nmea_msgs::msg::Sentence;
using MarkerMsg = visualization_msgs::msg::Marker;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using PointMsg = geometry_msgs::msg::Point;
using RosTime = rclcpp::Time;

template <typename MsgT>
using PublisherType = typename rclcpp::Publisher<MsgT>::SharedPtr;

template <typename MsgT>
using SubscriptionType = typename rclcpp::Subscription<MsgT>::SharedPtr;
#else
using OdometryMsg = nav_msgs::Odometry;
using SentenceMsg = nmea_msgs::Sentence;
using MarkerMsg = visualization_msgs::Marker;
using TransformStampedMsg = geometry_msgs::TransformStamped;
using PointMsg = geometry_msgs::Point;
using RosTime = ros::Time;

template <typename MsgT>
using PublisherType = ros::Publisher;

template <typename MsgT>
using SubscriptionType = ros::Subscriber;
#endif

/**
 * @brief 融合ROS节点
 * 支持ROS1和ROS2，通过条件编译切换
 */
class FusionNode {
public:
#if defined(ODIN_ROS2)
    explicit FusionNode(const rclcpp::Node::SharedPtr& node) : node_(node) {
        loadParameters();
        advertiseTopics();
        subscribeTopics();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        fusion_ = std::make_unique<NewFusion>(new_config_);
        logInfo("FusionNode initialized");
    }
#else
    FusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        loadParameters();
        advertiseTopics();
        subscribeTopics();
        fusion_ = std::make_unique<NewFusion>(new_config_);
        logInfo("FusionNode initialized");
    }
#endif

    ~FusionNode() = default;

private:
    // ========== Parameter Handling ==========
    template <typename T>
    T getParam(const std::string& name, const T& default_value) {
#if defined(ODIN_ROS2)
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_value);
        }
        T value = default_value;
        node_->get_parameter(name, value);
        return value;
#else
        return pnh_.param(name, default_value);
#endif
    }

    void loadParameters() {
        config_file_ = getParam<std::string>("config_file", std::string(""));
        try {
            const FusionNodeParams params = FusionNodeParams::LoadFromYamlFile(config_file_);
            new_config_ = params.algorithm;
            input_lla_crs_ = params.input_lla_crs;
            output_project_enable_ = params.output_project_enable;
            output_project_target_crs_ = params.output_project_target_crs;
        } catch (const std::exception& e) {
            logError("Failed to load config_file '%s': %s", config_file_.c_str(), e.what());
            new_config_ = NewFusion::Config();
        }

        projected_odom_topic_ = getParam<std::string>("output_project_topic", std::string("/world/odom_projected"));
        projected_frame_id_ = getParam<std::string>("output_project_frame_id", std::string("world_proj"));

        if (!enu_converter_.init(input_lla_crs_)) {
            logWarn("Failed to init ENU converter with input_lla_crs=%s, fallback to EPSG:4326", input_lla_crs_.c_str());
            input_lla_crs_ = "EPSG:4326";
            (void)enu_converter_.init(input_lla_crs_);
        }

        if (output_project_enable_) {
            if (output_project_target_crs_.empty()) {
                logWarn("output_project_enable is true but output_project_target_crs is empty, disable projection.");
                output_project_enable_ = false;
            } else if (!proj_converter_.init(input_lla_crs_, output_project_target_crs_)) {
                logWarn("Failed to init PROJ converter from %s to %s, disable projection.",
                        input_lla_crs_.c_str(), output_project_target_crs_.c_str());
                output_project_enable_ = false;
            }
        }

        logInfo("=== FusionNode Parameters ===");
        logInfo("  config_file: %s", config_file_.c_str());
        logInfo("  input_lla_crs: %s", input_lla_crs_.c_str());
    }

    // ========== Topic Advertising/Subscribing ==========
    void advertiseTopics() {
#if defined(ODIN_ROS2)
        fused_odom_pub_ = node_->create_publisher<OdometryMsg>("/world_enu/odom", rclcpp::QoS(100));
        gnss_marker_pub_ = node_->create_publisher<MarkerMsg>("/world_enu/gnss_marker", rclcpp::QoS(10));
        fused_odom_projected_pub_ = node_->create_publisher<OdometryMsg>(projected_odom_topic_, rclcpp::QoS(100));
#else
        fused_odom_pub_ = nh_.advertise<OdometryMsg>("/world_enu/odom", 100);
        gnss_marker_pub_ = nh_.advertise<MarkerMsg>("/world_enu/gnss_marker", 10);
        fused_odom_projected_pub_ = nh_.advertise<OdometryMsg>(projected_odom_topic_, 100);
#endif
    }

    void subscribeTopics() {
#if defined(ODIN_ROS2)
        odom_sub_ = node_->create_subscription<OdometryMsg>(
            "/odin1/odometry", rclcpp::QoS(200),
            std::bind(&FusionNode::odomCallback, this, std::placeholders::_1));
        gga_sub_ = node_->create_subscription<SentenceMsg>(
            "/odin1/nmea_sentence", rclcpp::QoS(50),
            std::bind(&FusionNode::ggaCallback, this, std::placeholders::_1));
#else
        odom_sub_ = nh_.subscribe("/odin1/odometry", 200, &FusionNode::odomCallbackRos1, this);
        gga_sub_ = nh_.subscribe("/odin1/nmea_sentence", 50, &FusionNode::ggaCallbackRos1, this);
#endif
    }

    // ========== Publishing Helpers ==========
    template <typename MsgT>
    void publish(PublisherType<MsgT>& pub, const MsgT& msg) {
#if defined(ODIN_ROS2)
        if (pub) { pub->publish(msg); }
#else
        pub.publish(msg);
#endif
    }

    // ========== Logging ==========
    template <typename... Args>
    void logInfo(const char* fmt, Args... args) const {
#if defined(ODIN_ROS2)
        RCLCPP_INFO(node_->get_logger(), fmt, args...);
#else
        ROS_INFO(fmt, args...);
#endif
    }

    template <typename... Args>
    void logWarn(const char* fmt, Args... args) const {
#if defined(ODIN_ROS2)
        RCLCPP_WARN(node_->get_logger(), fmt, args...);
#else
        ROS_WARN(fmt, args...);
#endif
    }

    template <typename... Args>
    void logError(const char* fmt, Args... args) const {
#if defined(ODIN_ROS2)
        RCLCPP_ERROR(node_->get_logger(), fmt, args...);
#else
        ROS_ERROR(fmt, args...);
#endif
    }

    template <typename... Args>
    void logWarnThrottle(double period_sec, const char* fmt, Args... args) {
#if defined(ODIN_ROS2)
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(),
                             static_cast<int>(period_sec * 1000), fmt, args...);
#else
        ROS_WARN_THROTTLE(period_sec, fmt, args...);
#endif
    }

    // ========== Time Conversion ==========
    double toSeconds(const RosTime& stamp) const {
#if defined(ODIN_ROS2)
        return stamp.seconds();
#else
        return stamp.toSec();
#endif
    }

    // ========== Callbacks ==========
#if defined(ODIN_ROS2)
    void odomCallback(const OdometryMsg::SharedPtr odom_msg) {
        processOdom(*odom_msg, odom_msg->header.stamp);
    }
    void ggaCallback(const SentenceMsg::SharedPtr msg) {
        processGga(*msg, msg->header.stamp);
    }
#else
    void odomCallbackRos1(const OdometryMsg::ConstPtr& odom_msg) {
        processOdom(*odom_msg, odom_msg->header.stamp);
    }
    void ggaCallbackRos1(const SentenceMsg::ConstPtr& msg) {
        processGga(*msg, msg->header.stamp);
    }
#endif

    // ========== Core Processing (ROS-agnostic) ==========
    void processOdom(const OdometryMsg& odom_msg, const RosTime& stamp) {
        OdomData odom = convertOdomMsg(odom_msg);
        fusion_->addOdom(odom, false);

        if (!fusion_->isInitialized()) { return; }

        const Eigen::Matrix4d world_T_odom = fusion_->getWorldToOdom();
        const Eigen::Matrix4d& odom_T_body = odom.pose;
        const Eigen::Matrix4d world_T_body = world_T_odom * odom_T_body;

        OdometryMsg fused_odom_msg;
        fused_odom_msg.header.stamp = stamp;
        fused_odom_msg.header.frame_id = "world";
        fused_odom_msg.child_frame_id = "base_link";

        fused_odom_msg.pose.pose.position.x = world_T_body(0, 3);
        fused_odom_msg.pose.pose.position.y = world_T_body(1, 3);
        fused_odom_msg.pose.pose.position.z = world_T_body(2, 3);

        Eigen::Matrix3d rot = world_T_body.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rot);
        q.normalize();
        fused_odom_msg.pose.pose.orientation.w = q.w();
        fused_odom_msg.pose.pose.orientation.x = q.x();
        fused_odom_msg.pose.pose.orientation.y = q.y();
        fused_odom_msg.pose.pose.orientation.z = q.z();

        publish(fused_odom_pub_, fused_odom_msg);
        publishProjectedOdom(fused_odom_msg);
        publishTF(stamp);
    }

    void processGga(const SentenceMsg& msg, const RosTime& stamp) {
        const double ros_timestamp = toSeconds(stamp);

        GGAData gga = GGAParser::parse(msg.sentence);
        if (!gga.valid) { return; }

        if (!std::isfinite(gga.lat) || !std::isfinite(gga.lon) || !std::isfinite(gga.alt) ||
            !std::isfinite(gga.hdop) || !std::isfinite(gga.timestamp)) {
            logWarnThrottle(1.0, "Skip GGA: non-finite value detected (lat/lon/alt/hdop/utc)");
            return;
        }

        if (!enu_converter_.isInitialized()) {
            enu_converter_.setOrigin(gga.lat, gga.lon, gga.alt);
            logInfo("ENU origin set: lat=%.7f, lon=%.7f, alt=%.2f", gga.lat, gga.lon, gga.alt);
        }

        Eigen::Vector3d enu = enu_converter_.toENU(gga.lat, gga.lon, gga.alt);
        if (!std::isfinite(enu.x()) || !std::isfinite(enu.y()) || !std::isfinite(enu.z())) {
            logWarnThrottle(1.0, "Skip GNSS publish: ENU is non-finite (%.3f %.3f %.3f)",
                           enu.x(), enu.y(), enu.z());
            return;
        }

        MarkerMsg marker = makeGnssMarker(stamp, enu, gga);
        publish(gnss_marker_pub_, marker);

        GPSData gps;
        gps.timestamp = ros_timestamp;
        gps.lat = gga.lat;
        gps.lon = gga.lon;
        gps.alt = gga.alt;
        gps.quality = gga.quality;
        gps.hdop = gga.hdop;
        gps.num_satellites = gga.num_satellites;
        gps.valid = gga.valid;
        gps.enu = enu;
        fusion_->addGPS(gps);
    }

    void publishProjectedOdom(const OdometryMsg& enu_odom_msg) {
        if (!output_project_enable_ || !proj_converter_.isInitialized() || !enu_converter_.isInitialized()) {
            return;
        }

        const Eigen::Vector3d enu(enu_odom_msg.pose.pose.position.x,
                                  enu_odom_msg.pose.pose.position.y,
                                  enu_odom_msg.pose.pose.position.z);

        double lat = 0.0, lon = 0.0, alt = 0.0;
        enu_converter_.toLLA(enu, lat, lon, alt);

        double out_x = 0.0, out_y = 0.0, out_z = 0.0;
        if (!proj_converter_.projectDeg(lat, lon, alt, out_x, out_y, out_z)) { return; }

        OdometryMsg projected = enu_odom_msg;
        projected.header.frame_id = projected_frame_id_;
        projected.pose.pose.position.x = out_x;
        projected.pose.pose.position.y = out_y;
        projected.pose.pose.position.z = out_z;
        publish(fused_odom_projected_pub_, projected);
    }

    void publishTF(const RosTime& stamp) {
        Eigen::Matrix4d world_T_odom = fusion_->getWorldToOdom();

        TransformStampedMsg tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = "odom";

        tf_msg.transform.translation.x = world_T_odom(0, 3);
        tf_msg.transform.translation.y = world_T_odom(1, 3);
        tf_msg.transform.translation.z = world_T_odom(2, 3);

        Eigen::Matrix3d rot = world_T_odom.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rot);
        q.normalize();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();

#if defined(ODIN_ROS2)
        tf_broadcaster_->sendTransform(tf_msg);
#else
        tf_broadcaster_.sendTransform(tf_msg);
#endif
    }

    // ========== Message Conversion ==========
    OdomData convertOdomMsg(const OdometryMsg& msg) const {
        OdomData odom;
#if defined(ODIN_ROS2)
        odom.timestamp = rclcpp::Time(msg.header.stamp).seconds();
#else
        odom.timestamp = msg.header.stamp.toSec();
#endif
        Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        q.normalize();
        Eigen::Vector3d t(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

        odom.pose = Eigen::Matrix4d::Identity();
        odom.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        odom.pose.block<3, 1>(0, 3) = t;
        return odom;
    }

    MarkerMsg makeGnssMarker(const RosTime& stamp, const Eigen::Vector3d& enu, const GGAData& gga) {
        MarkerMsg m;
        m.header.stamp = stamp;
        m.header.frame_id = "world";
        m.ns = "gnss";
        m.id = 0;
        m.action = MarkerMsg::ADD;
        m.pose.orientation.w = 1.0;
        m.pose.position.x = enu.x();
        m.pose.position.y = enu.y();
        m.pose.position.z = enu.z();
        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m.scale.z = 0.5;

        m.type = MarkerMsg::SPHERE;

        if (gga.quality == 5) {
            m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0;  // blue
        } else if (gga.quality == 4) {
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;  // green
        } else if (gga.quality == 1 || gga.quality == 2) {
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;  // yellow
        } else {
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;  // red
        }

        double s = 2.0;
        if (gga.quality == 4 || gga.quality == 5) {
            s = std::min(std::max(0.3, 0.15 * gga.hdop), 2.0);
        }
        m.scale.x = s; m.scale.y = s; m.scale.z = s;
        return m;
    }

    // ========== Member Variables ==========
#if defined(ODIN_ROS2)
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
#else
    ros::NodeHandle nh_, pnh_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
#endif

    std::unique_ptr<FusionBase> fusion_;
    NewFusion::Config new_config_;
    ENUConverter enu_converter_;
    ProjConverter proj_converter_;

    SubscriptionType<OdometryMsg> odom_sub_;
    SubscriptionType<SentenceMsg> gga_sub_;

    PublisherType<OdometryMsg> fused_odom_pub_;
    PublisherType<OdometryMsg> fused_odom_projected_pub_;
    PublisherType<MarkerMsg> gnss_marker_pub_;

    std::string config_file_;
    std::string input_lla_crs_ = "EPSG:4326";
    bool output_project_enable_ = false;
    std::string output_project_target_crs_;
    std::string projected_odom_topic_ = "/world/odom_projected";
    std::string projected_frame_id_ = "world_proj";
};

}  // namespace odin_rtk_fusion

// ========== Main Functions ==========
#if defined(ODIN_ROS2)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    INIT_SPDLOG_CLEAR((WORK_SPACE_PATH + "/log/log.txt").c_str());
    SET_LOG_LEVEL(spdlog::level::debug);

    try {
        auto node = std::make_shared<rclcpp::Node>("fusion_node");
        auto driver = std::make_unique<odin_rtk_fusion::FusionNode>(node);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
#else
int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    INIT_SPDLOG_CLEAR((WORK_SPACE_PATH + "/log/log.txt").c_str());
    SET_LOG_LEVEL(spdlog::level::debug);

    try {
        odin_rtk_fusion::FusionNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    return 0;
}
#endif
