/*
 * Copyright 2026 Manifold Tech Ltd.(manifoldtech.cn)
 * Licensed under the Apache License, Version 2.0
 */

#include "fusion/alignment/alignment_fusion.hpp"
#include <cmath>
#include <gtsam/geometry/Pose3.h>

namespace odin_rtk_fusion {

AlignmentFusion::AlignmentFusion(const Config& config)
    : config_(config),
      odom_buffer_(),
      gps_buffer_(static_cast<size_t>(config.init_gps_buffer_size)) {
}

void AlignmentFusion::addOdom(const OdomData& odom, bool is_high_freq) {
    (void)is_high_freq;
    std::lock_guard<std::mutex> lock(mutex_);

    odom_buffer_.push(odom.timestamp, gtsam::Pose3(odom.pose));
    last_odom_pose_ = odom.pose;
    last_odom_timestamp_ = odom.timestamp;
    have_odom_ = true;
}

void AlignmentFusion::addGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(mutex_);

    const double timestamp = gps.timestamp + gps_time_offset_;
    const Eigen::Vector3d& enu = gps.enu;
    const int quality = gps.quality;

    if (!have_odom_) {
        LOG_WARN("[AlignmentFusion] addGPS: no odom yet, skip");
        return;
    }

    if (quality == 0) {
        LOG_WARN("[AlignmentFusion] addGPS: quality=0, skip");
        return;
    }

    // ========== Initialization phase ==========
    if (state_ != State::kRunning) {
        LOG_INFO("[AlignmentFusion] addGPS: initializing, gps_buf={} odom_buf={}",
                 gps_buffer_.size(), odom_buffer_.size());
        gps_buffer_.push(timestamp, enu, quality, gps.hdop);

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

        // Seed alignment buffer with init data
        const Eigen::Vector3d odom_t = init_result->odom_ref.translation();
        AlignPair p;
        p.gps_xy = init_result->gps_enu.head<2>();
        p.odom_xy = odom_t.head<2>();
        p.gps_z = init_result->gps_enu.z();
        p.odom_z = odom_t.z();
        align_pairs_.push_back(p);

        solveWTo();

        state_ = State::kRunning;
        gps_buffer_.clear();
        odom_buffer_.clear();

        LOG_INFO("[AlignmentFusion] Initialized: yaw={:.2f}deg wTo=[{:.3f} {:.3f} {:.3f}]",
                 std::atan2(world_T_odom_(1,0), world_T_odom_(0,0)) * 180.0 / M_PI,
                 world_T_odom_(0,3), world_T_odom_(1,3), world_T_odom_(2,3));
        return;
    }

    // ========== Running phase: collect GPS-odom pairs, solve wTo ==========

    // Only use RTK Fix / Float
    if (quality != 4 && quality != 5) {
        return;
    }

    // Find nearest odom to this GPS timestamp
    auto odom_opt = odom_buffer_.nearest(timestamp);
    if (!odom_opt) {
        return;
    }
    if (std::abs(odom_opt->timestamp - timestamp) > config_.max_gps_match_dt) {
        return;
    }

    const Eigen::Matrix4d odom_pose = odom_opt->pose.matrix();
    const Eigen::Vector3d odom_t = odom_pose.block<3,1>(0,3);

    // Add pair to sliding window
    AlignPair p;
    p.gps_xy = enu.head<2>();
    p.odom_xy = odom_t.head<2>();
    p.gps_z = enu.z();
    p.odom_z = odom_t.z();
    align_pairs_.push_back(p);

    // Keep window bounded
    if (align_pairs_.size() > kMaxAlignPairs) {
        align_pairs_.erase(align_pairs_.begin());
    }

    // Solve for optimal wTo from all pairs
    solveWTo();

    // Diagnostic
    const Eigen::Matrix4d pred = world_T_odom_ * odom_pose;
    LOG_INFO("[AlignmentFusion] n={} pred=[{:.3f} {:.3f} {:.3f}] gps=[{:.3f} {:.3f} {:.3f}] err=[{:.3f} {:.3f} {:.3f}]",
             align_pairs_.size(),
             pred(0,3), pred(1,3), pred(2,3),
             enu.x(), enu.y(), enu.z(),
             pred(0,3)-enu.x(), pred(1,3)-enu.y(), pred(2,3)-enu.z());
}

// Solve optimal wTo (yaw + translation) from GPS-odom pairs using least squares
// 2D Procrustes: gps_xy = Rz(yaw) * odom_xy + t_xy
void AlignmentFusion::solveWTo() {
    const size_t n = align_pairs_.size();
    if (n < 1) return;

    // 1. Compute centroids
    Eigen::Vector2d gps_c = Eigen::Vector2d::Zero();
    Eigen::Vector2d odom_c = Eigen::Vector2d::Zero();
    double gz_sum = 0, oz_sum = 0;
    for (const auto& p : align_pairs_) {
        gps_c += p.gps_xy;
        odom_c += p.odom_xy;
        gz_sum += p.gps_z;
        oz_sum += p.odom_z;
    }
    gps_c /= static_cast<double>(n);
    odom_c /= static_cast<double>(n);

    // 2. Compute cross-correlation for yaw (centered points)
    double s00 = 0, s01 = 0, s10 = 0, s11 = 0;
    for (const auto& p : align_pairs_) {
        const Eigen::Vector2d g = p.gps_xy - gps_c;
        const Eigen::Vector2d o = p.odom_xy - odom_c;
        s00 += o.x() * g.x();
        s01 += o.x() * g.y();
        s10 += o.y() * g.x();
        s11 += o.y() * g.y();
    }

    // 3. Optimal yaw: atan2(s01 - s10, s00 + s11)
    const double yaw = std::atan2(s01 - s10, s00 + s11);
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    // 4. Translation: t = gps_c - R * odom_c
    const double tx = gps_c.x() - (cy * odom_c.x() - sy * odom_c.y());
    const double ty = gps_c.y() - (sy * odom_c.x() + cy * odom_c.y());
    const double tz = gz_sum / n - oz_sum / n;

    // 5. Build wTo matrix
    world_T_odom_ = Eigen::Matrix4d::Identity();
    world_T_odom_(0,0) =  cy; world_T_odom_(0,1) = -sy;
    world_T_odom_(1,0) =  sy; world_T_odom_(1,1) =  cy;
    world_T_odom_(0,3) = tx;
    world_T_odom_(1,3) = ty;
    world_T_odom_(2,3) = tz;
}

bool AlignmentFusion::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == State::kRunning;
}

Eigen::Matrix4d AlignmentFusion::getWorldToOdom() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return world_T_odom_;
}

} // namespace odin_rtk_fusion
