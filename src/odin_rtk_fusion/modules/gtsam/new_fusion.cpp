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
#include "new_fusion.hpp"

#include <iostream>
#include <cmath>

namespace odin_rtk_fusion {

// ============================================================================
// Anonymous namespace - utility functions
// ============================================================================
namespace {

// 将天线 ENU 坐标转换为 body 原点 ENU 坐标
// antenna_enu: 天线在 world 坐标系下的 ENU 位置
// world_R_body: body 在 world 坐标系下的旋转
// lever_arm: body 坐标系下天线相对于 body 原点的偏移
gtsam::Point3 antennaEnuToBodyEnu(const Eigen::Vector3d& antenna_enu,
                                   const gtsam::Rot3& world_R_body,
                                   const Eigen::Vector3d& lever_arm) {
    const gtsam::Point3 lever_body(lever_arm.x(), lever_arm.y(), lever_arm.z());
    const gtsam::Point3 lever_world = world_R_body.rotate(lever_body);
    return gtsam::Point3(antenna_enu.x() - lever_world.x(),
                         antenna_enu.y() - lever_world.y(),
                         antenna_enu.z() - lever_world.z());
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

NewFusion::NewFusion(const Config& config)
    : config_(config),
      gps_buffer_(static_cast<size_t>(config.init_gps_buffer_size)) {
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
}

// ============================================================================
// Public Interface
// ============================================================================

void NewFusion::addOdom(const OdomData& odom, bool is_high_freq) {
    (void)is_high_freq;
    std::lock_guard<std::mutex> lock(mutex_);

    const gtsam::Pose3 cur_odom_pose(gtsam::Pose3(odom.pose));
    have_odom_ = true;

    // 缓存 odom 数据（初始化和运行时共用同一个 buffer）
    odom_buffer_.push(odom.timestamp, cur_odom_pose);
}

void NewFusion::addGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 应用时间偏移补偿（初始化前 gps_time_offset_ 为 0）
    const double timestamp = gps.timestamp + gps_time_offset_;
    const Eigen::Vector3d& enu = gps.enu;
    const int quality = gps.quality;
    const double hdop = gps.hdop;

    if (!have_odom_) {
        return;
    }

    // quality 0 无效数据直接丢弃
    if (quality == 0) {
        return;
    }

    // ========== 初始化阶段 ==========
    if (state_ != State::kRunning) {
        gps_buffer_.push(timestamp, enu, quality, hdop);

        // 尝试计算初始 yaw 偏移
        YawInitConfig yaw_config;
        yaw_config.min_gps_displacement = config_.init_min_gps_displacement;
        yaw_config.min_odom_displacement = config_.init_min_odom_displacement;
        yaw_config.min_samples = 2;

        auto init_result = computeInitYawOffset(odom_buffer_, gps_buffer_, yaw_config);
        if (!init_result) {
            return;
        }

        init_yaw_offset_ = init_result->yaw_offset;
        gps_time_offset_ = init_result->time_offset;
        initializeWithFirstGps(*init_result);
        gps_buffer_.clear();
        odom_buffer_.clear();
        return;
    }

    // ========== 运行阶段 ==========
    
    // 获取自上次节点以来的所有 odom 数据
    auto odom_samples = odom_buffer_.range(last_node_timestamp_, timestamp);
    if (odom_samples.empty()) {
        // 没有新的 odom 数据，尝试用最近的
        auto odom_opt = odom_buffer_.nearest(timestamp);
        if (!odom_opt) {
            return;
        }
        const double time_diff = std::abs(odom_opt->timestamp - timestamp);
        if (time_diff > config_.max_gps_match_dt) {
            return;
        }
        odom_samples.push_back(*odom_opt);
    }

    const gtsam::Pose3 world_T_odom(world_T_odom_);

    // 为每个 odom 创建节点和因子
    for (size_t i = 0; i < odom_samples.size(); ++i) {
        const auto& sample = odom_samples[i];
        const gtsam::Pose3& cur_odom_pose = sample.pose;

        // 添加 odom 因子 (连接到上一个节点)
        if (node_count_ > 0) {
            addOdomFactor(node_count_ - 1, node_count_, last_node_odom_pose_, cur_odom_pose);
        }

        // 添加节点初值
        const gtsam::Pose3 predicted = world_T_odom * cur_odom_pose;
        if (!initial_.exists(X(node_count_))) {
            initial_.insert(X(node_count_), predicted);
        }

        // 记录节点状态
        last_node_odom_pose_ = cur_odom_pose;
        last_node_timestamp_ = sample.timestamp;

        // 在最后一个节点上添加 GPS 因子
        if (i == odom_samples.size() - 1) {
            addGpsFactor(node_count_, cur_odom_pose, enu, quality, hdop);

            const auto& odom_world = predicted.translation();
            // LOG_INFO("[POS_LOG] t={:.6f} GPS {:.4f} {:.4f} {:.4f} ODOM {:.4f} {:.4f} {:.4f}",
            //          timestamp, enu.x(), enu.y(), enu.z(),
            //          odom_world.x(), odom_world.y(), odom_world.z());
        }

        node_count_++;
    }

    // 清理已处理的 odom 数据
    odom_buffer_.popUntil(last_node_timestamp_);

    optimize();
    marginalize();
}

bool NewFusion::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == State::kRunning;
}

Eigen::Matrix4d NewFusion::getWorldToOdom() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return world_T_odom_;
}

// ============================================================================
// Factor Graph Operations
// ============================================================================

void NewFusion::addOdomFactor(int from_id, int to_id, const gtsam::Pose3& from_pose, const gtsam::Pose3& to_pose) {
    const gtsam::Pose3 delta_odom = from_pose.inverse() * to_pose;

    const gtsam::Point3 dp = delta_odom.translation();
    const double dist = std::max(1e-3, std::sqrt(dp.x() * dp.x() + dp.y() * dp.y() + dp.z() * dp.z()));
    const double dtheta = std::max(1e-3, gtsam::Rot3::Logmap(delta_odom.rotation()).norm());
    const double rot_sigma = config_.odom_rotation_sigma * dtheta;
    const double pos_sigma = config_.odom_position_sigma * dist;

    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma, pos_sigma, pos_sigma, pos_sigma).finished());

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(from_id), X(to_id), delta_odom, odom_noise));
}

void NewFusion::addGpsFactor(int node_id, const gtsam::Pose3& odom_pose, const Eigen::Vector3d& enu, int quality, double hdop) {
    // 考虑 lever arm 将天线位置转换到 body 原点
    const gtsam::Pose3 world_T_odom(world_T_odom_);
    const gtsam::Rot3 world_R_body = (world_T_odom * odom_pose).rotation();
    gtsam::Point3 body_enu = antennaEnuToBodyEnu(enu, world_R_body, config_.initial_lever_arm);

    // 噪声：根据GPS质量调整
    const double gps_sigma = config_.gps_base_sigma * hdop;
    gtsam::Vector sigma3(3);
    if (quality == 4) {
        // RTK Fix
        sigma3 << gps_sigma, gps_sigma, gps_sigma;
    } else if (quality == 5) {
        // RTK Float
        sigma3 << gps_sigma * 2, gps_sigma * 2, 1e2;
        body_enu = gtsam::Point3(body_enu.x(), body_enu.y(), 0.0);
    } else {
        // GPS/DGPS
        sigma3 << gps_sigma * 10, gps_sigma * 10, 1e2;
        body_enu = gtsam::Point3(body_enu.x(), body_enu.y(), 0.0);
    }

    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(sigma3);
    graph_.add(gtsam::GPSFactor(X(node_id), body_enu, gps_noise));
}

void NewFusion::optimize() {
    if (graph_.empty() && initial_.empty()) {
        return;
    }

    try {
        isam_->update(graph_, initial_);
        graph_.resize(0);
        initial_.clear();

        gtsam::Values result = isam_->calculateEstimate();
        if (node_count_ > 0 && result.exists(X(node_count_ - 1))) {
            last_key_world_pose_ = result.at<gtsam::Pose3>(X(node_count_ - 1));
            // 更新 world_T_odom
            world_T_odom_ = (last_key_world_pose_ * last_node_odom_pose_.inverse()).matrix();
        }
    } catch (const std::exception& e) {
        std::cerr << "[NewFusion] Optimization failed: " << e.what() << std::endl;
        graph_.resize(0);
        initial_.clear();
    }
}

void NewFusion::marginalize() {
    const int keep_last_n = config_.max_nodes;
    if (node_count_ <= keep_last_n) {
        return;
    }

    const int oldest_to_keep = node_count_ - keep_last_n;
    if (oldest_to_keep <= last_marginalized_id_ + 1) {
        return;
    }

    gtsam::FastList<gtsam::Key> to_marginalize;
    for (int i = last_marginalized_id_ + 1; i < oldest_to_keep; ++i) {
        to_marginalize.push_back(X(i));
    }

    if (to_marginalize.empty()) {
        return;
    }

    try {
        isam_->marginalizeLeaves(to_marginalize);
        last_marginalized_id_ = oldest_to_keep - 1;
    } catch (const std::exception& e) {
        std::cerr << "[NewFusion] Marginalization failed: " << e.what() << std::endl;
    }
}

void NewFusion::initializeWithFirstGps(const YawInitResult& init_result) {
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
    graph_.resize(0);
    initial_.clear();

    node_count_ = 1;

    const gtsam::Pose3& odom_ref = init_result.odom_ref;
    const Eigen::Vector3d& antenna_enu = init_result.gps_enu;
    const gtsam::Rot3 world_R_odom = gtsam::Rot3::Rz(init_yaw_offset_);
    const gtsam::Point3 odom_t_body = odom_ref.translation();

    // 天线 ENU → body ENU
    const gtsam::Rot3 world_R_body = world_R_odom * odom_ref.rotation();
    const gtsam::Point3 world_t_body = antennaEnuToBodyEnu(antenna_enu, world_R_body, config_.initial_lever_arm);

    // 计算 world_T_odom
    const gtsam::Point3 world_t_odom_adjusted = world_t_body - world_R_odom.rotate(odom_t_body);
    const gtsam::Pose3 world_T_odom(world_R_odom, world_t_odom_adjusted);
    const gtsam::Pose3 x0 = world_T_odom * odom_ref;
    initial_.insert(X(0), x0);

    auto gauge_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1, 1, config_.init_yaw_prior_sigma, 1e3, 1e3, 1e3).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3(world_R_odom, gtsam::Point3()), gauge_noise));

    // 同时加一个 GPSFactor 锚定位置
    const double gps_sigma = std::max(0.5, config_.gps_base_sigma);
    gtsam::Vector sigma3(3);
    sigma3 << gps_sigma, gps_sigma, gps_sigma;
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(sigma3);
    graph_.add(gtsam::GPSFactor(X(0), gtsam::Point3(world_t_body.x(), world_t_body.y(), world_t_body.z()), gps_noise));

    last_node_odom_pose_ = odom_ref;
    last_node_timestamp_ = init_result.gps_timestamp;

    last_key_world_pose_ = x0;

    state_ = State::kRunning;

    optimize();
}

} // namespace odin_rtk_fusion
