/*
 * Copyright 2026 Manifold Tech Ltd.(manifoldtech.cn)
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
#include "fusion/gtsam/graph_fusion.hpp"

#include <cmath>

namespace odin_rtk_fusion {

// ============================================================================
// Anonymous namespace - utility functions
// ============================================================================
namespace {

// Convert antenna ENU coordinates to body origin ENU coordinates
// antenna_enu: antenna position in world ENU frame
// world_R_body: body rotation in world frame
// lever_arm: antenna offset from body origin in body frame
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

GraphFusion::GraphFusion(const Config& config)
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

void GraphFusion::addOdom(const OdomData& odom, bool is_high_freq) {
    (void)is_high_freq;
    std::lock_guard<std::mutex> lock(mutex_);

    const gtsam::Pose3 cur_odom_pose(gtsam::Pose3(odom.pose));
    have_odom_ = true;

    // Buffer odom data (shared buffer for init and runtime)
    odom_buffer_.push(odom.timestamp, cur_odom_pose);
}

void GraphFusion::addGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Apply time offset compensation (gps_time_offset_ is 0 before init)
    const double timestamp = gps.timestamp + gps_time_offset_;
    const Eigen::Vector3d& enu = gps.enu;
    const int quality = gps.quality;
    const double hdop = gps.hdop;

    if (!have_odom_) {
        return;
    }

    // Discard invalid data with quality 0
    if (quality == 0) {
        return;
    }

    // ========== Initialization phase ==========
    if (state_ != State::kRunning) {
        gps_buffer_.push(timestamp, enu, quality, hdop);

        // Try to compute initial yaw offset
        YawInitConfig yaw_config;
        yaw_config.min_gps_displacement = config_.init_min_gps_displacement;
        yaw_config.min_odom_displacement = config_.init_min_odom_displacement;
        yaw_config.min_samples = 2;
        if (config_.disable_gps_time_offset_estimation) {
            yaw_config.time_offset_range = 0.0;
        }

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

    // ========== Running phase ==========
    
    // Get all odom data since last node
    auto odom_samples = odom_buffer_.range(last_node_timestamp_, timestamp);
    if (odom_samples.empty()) {
        // No new odom data, try using the nearest
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

    // Create nodes and factors for each odom sample
    for (size_t i = 0; i < odom_samples.size(); ++i) {
        const auto& sample = odom_samples[i];
        const gtsam::Pose3& cur_odom_pose = sample.pose;

        // Add odom factor (connect to previous node)
        if (node_count_ > 0) {
            addOdomFactor(node_count_ - 1, node_count_, last_node_odom_pose_, cur_odom_pose);
        }

        // Add initial node value
        const gtsam::Pose3 predicted = world_T_odom * cur_odom_pose;
        if (!initial_.exists(X(node_count_))) {
            initial_.insert(X(node_count_), predicted);
        }

        // Record node state
        last_node_odom_pose_ = cur_odom_pose;
        last_node_timestamp_ = sample.timestamp;

        // Add GPS factor on the last node
        if (i == odom_samples.size() - 1) {
            // Log raw inputs BEFORE optimization
            const gtsam::Point3 odom_t = cur_odom_pose.translation();
            const gtsam::Point3 pred_t = (world_T_odom * cur_odom_pose).translation();
            LOG_INFO("[INPUT] node x{}: odom=[{:.3f} {:.3f} {:.3f}] gps_enu=[{:.3f} {:.3f} {:.3f}] predicted=[{:.3f} {:.3f} {:.3f}]",
                     node_count_,
                     odom_t.x(), odom_t.y(), odom_t.z(),
                     enu.x(), enu.y(), enu.z(),
                     pred_t.x(), pred_t.y(), pred_t.z());
            addGpsFactor(node_count_, cur_odom_pose, enu, quality, hdop);
        }

        node_count_++;
    }

    // Clean up processed odom data
    odom_buffer_.popUntil(last_node_timestamp_);

    optimize();

    // Log optimized result AFTER optimization
    if (node_count_ > 0) {
        const gtsam::Point3 opt_t = last_key_world_pose_.translation();
        const gtsam::Rot3 opt_r = last_key_world_pose_.rotation();
        LOG_INFO("[OUTPUT] node x{}: optimized=[{:.3f} {:.3f} {:.3f}] yaw={:.2f}deg",
                 node_count_ - 1,
                 opt_t.x(), opt_t.y(), opt_t.z(),
                 opt_r.yaw() * 180.0 / M_PI);
    }
    marginalize();
}

bool GraphFusion::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == State::kRunning;
}

Eigen::Matrix4d GraphFusion::getWorldToOdom() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return world_T_odom_;
}

// ============================================================================
// Factor Graph Operations
// ============================================================================

void GraphFusion::addOdomFactor(int from_id, int to_id, const gtsam::Pose3& from_pose, const gtsam::Pose3& to_pose) {
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

void GraphFusion::addGpsFactor(int node_id, const gtsam::Pose3& odom_pose, const Eigen::Vector3d& enu, int quality, double hdop) {
    // Apply lever arm to convert antenna position to body origin
    const gtsam::Pose3 world_T_odom(world_T_odom_);
    const gtsam::Rot3 world_R_body = (world_T_odom * odom_pose).rotation();
    gtsam::Point3 body_enu = antennaEnuToBodyEnu(enu, world_R_body, config_.initial_lever_arm);

    // Noise: adjust based on GPS quality
    const double gps_sigma = config_.gps_base_sigma * hdop;
    std::cout << "gps_sigma: " << gps_sigma << " hdop: " << hdop << "   "<< std::endl;
    gtsam::Vector sigma3(3);
    if (quality == 4) {
        // RTK Fix — z uses configurable sigma
        sigma3 << gps_sigma, gps_sigma, gps_sigma * 2;
    } else if (quality == 5) {
        // RTK Float — weak z
        sigma3 << gps_sigma * 2, gps_sigma * 2, 1e2;
        body_enu = gtsam::Point3(body_enu.x(), body_enu.y(), 0.0);
    } else {
        // GPS/DGPS — weak z
        sigma3 << gps_sigma * 10, gps_sigma * 10, 1e2;
        body_enu = gtsam::Point3(body_enu.x(), body_enu.y(), 0.0);
    }

    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(sigma3);
    graph_.add(gtsam::GPSFactor(X(node_id), body_enu, gps_noise));
    return;
    // Store GPS body z for wTo z anchoring
    last_gps_body_z_ = body_enu.z();

    // Diagnostic: GPS residual (predicted - measurement)
    const gtsam::Point3 predicted_pos = (world_T_odom * odom_pose).translation();
    LOG_INFO("[DIAG] GPS residual: pred=[{:.3f} {:.3f} {:.3f}] meas=[{:.3f} {:.3f} {:.3f}] err=[{:.4f} {:.4f} {:.4f}]",
             predicted_pos.x(), predicted_pos.y(), predicted_pos.z(),
             body_enu.x(), body_enu.y(), body_enu.z(),
             predicted_pos.x()-body_enu.x(), predicted_pos.y()-body_enu.y(), predicted_pos.z()-body_enu.z());

    // GPS heading factor: provides absolute yaw constraint
    // Only update anchor when displacement >= 0.5m (works at any GPS rate)
    if (have_last_gps_ && quality == 4) {
        const double dx = enu.x() - last_gps_enu_.x();
        const double dy = enu.y() - last_gps_enu_.y();
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Use odom's roll/pitch for heading pose
        const gtsam::Rot3 odom_rot = (world_T_odom * odom_pose).rotation();

        if (dist > 0.5) {
            const double gps_heading = std::atan2(dy, dx);
            const gtsam::Rot3 heading_rot = gtsam::Rot3::Ypr(gps_heading,
                                                               odom_rot.pitch(),
                                                               odom_rot.roll());
            const gtsam::Pose3 heading_pose(heading_rot, body_enu);

            // Heading sigma: tighter when baseline longer (more reliable direction)
            const double heading_sigma = std::max(0.05, 0.3 / dist);
            auto heading_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 10.0, 10.0, heading_sigma, 1e3, 1e3, 1e3).finished());
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(node_id), heading_pose, heading_noise));

            LOG_INFO("[DIAG] GPS heading: {:.2f}deg dist={:.3f}m sigma={:.3f}rad",
                     gps_heading * 180.0 / M_PI, dist, heading_sigma);

            // Store last good heading
            last_gps_heading_ = gps_heading;
            last_odom_yaw_at_gps_ = odom_pose.rotation().yaw();
            have_gps_heading_ = true;

            // Update anchor only after successful heading computation
            last_gps_enu_ = enu;
        } else if (have_gps_heading_) {
            // GPS quasi-stationary: anchor yaw to odom-predicted heading
            // expected_yaw = last_gps_heading + odom rotation since then
            const double odom_yaw_now = odom_pose.rotation().yaw();
            double odom_dyaw = odom_yaw_now - last_odom_yaw_at_gps_;
            if (odom_dyaw > M_PI) odom_dyaw -= 2*M_PI;
            if (odom_dyaw < -M_PI) odom_dyaw += 2*M_PI;
            const double expected_yaw = last_gps_heading_ + odom_dyaw;

            const gtsam::Rot3 hold_rot = gtsam::Rot3::Ypr(expected_yaw,
                                                            odom_rot.pitch(),
                                                            odom_rot.roll());
            const gtsam::Pose3 hold_pose(hold_rot, body_enu);
            auto hold_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 10.0, 10.0, 0.1, 1e3, 1e3, 1e3).finished());
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(node_id), hold_pose, hold_noise));
            LOG_INFO("[DIAG] GPS heading HOLD: expected={:.2f}deg (gps_h={:.2f} + odom_d={:.2f})",
                     expected_yaw * 180.0 / M_PI, last_gps_heading_ * 180.0 / M_PI, odom_dyaw * 180.0 / M_PI);
        }
    } else if (quality == 4) {
        // First RTK Fix GPS: set anchor
        last_gps_enu_ = enu;
        have_last_gps_ = true;
    }
}

void GraphFusion::optimize() {
    if (graph_.empty() && initial_.empty()) {
        return;
    }

    try {
        // Diagnostic: capture old node position BEFORE update
        const int stable_id = std::max(0, node_count_ - 10);
        gtsam::Point3 old_stable_pos(0,0,0);
        bool has_stable = false;
        if (node_count_ > 10) {
            try {
                old_stable_pos = isam_->calculateEstimate<gtsam::Pose3>(X(stable_id)).translation();
                has_stable = true;
            } catch (...) {}
        }

        isam_->update(graph_, initial_);
        graph_.resize(0);
        initial_.clear();

        gtsam::Values result = isam_->calculateEstimate();
        if (node_count_ > 0 && result.exists(X(node_count_ - 1))) {
            last_key_world_pose_ = result.at<gtsam::Pose3>(X(node_count_ - 1));

            // Update world_T_odom: yaw-only (no pitch/roll)
            const Eigen::Matrix4d raw_wTo = (last_key_world_pose_ * last_node_odom_pose_.inverse()).matrix();
            const double wTo_yaw = std::atan2(raw_wTo(1,0), raw_wTo(0,0));
            const double cy = std::cos(wTo_yaw), sy = std::sin(wTo_yaw);
            const gtsam::Point3 world_t = last_key_world_pose_.translation();
            const gtsam::Point3 odom_t = last_node_odom_pose_.translation();
            world_T_odom_ = Eigen::Matrix4d::Identity();
            world_T_odom_(0,0) =  cy; world_T_odom_(0,1) = -sy;
            world_T_odom_(1,0) =  sy; world_T_odom_(1,1) =  cy;
            world_T_odom_(0,3) = world_t.x() - (cy * odom_t.x() - sy * odom_t.y());
            world_T_odom_(1,3) = world_t.y() - (sy * odom_t.x() + cy * odom_t.y());
            world_T_odom_(2,3) = world_t.z() - odom_t.z();
        }
    } catch (const std::exception& e) {
        LOG_ERROR("[GraphFusion] Optimization failed: {}, rebuilding ISAM2...", e.what());
        // graph_.resize(0);
        // initial_.clear();

        // // Rebuild ISAM2 from last good state
        // gtsam::ISAM2Params isam_params;
        // isam_params.relinearizeThreshold = 0.1;
        // isam_params.relinearizeSkip = 1;
        // isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

        // // Re-anchor with a prior on current best estimate
        // const double recover_sigma = std::max(1.0, config_.gps_base_sigma);
        // auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        //     (gtsam::Vector(6) << 0.1, 0.1, 0.1, recover_sigma, recover_sigma, recover_sigma).finished());
        // graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(node_count_ - 1), last_key_world_pose_, prior_noise));
        // initial_.insert(X(node_count_ - 1), last_key_world_pose_);
        // isam_->update(graph_, initial_);
        graph_.resize(0);
        initial_.clear();
    }
}

void GraphFusion::marginalize() {
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
        LOG_ERROR("[GraphFusion] Marginalization failed: {}", e.what());
    }
}

void GraphFusion::initializeWithFirstGps(const YawInitResult& init_result) {
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

    // Antenna ENU -> body ENU
    const gtsam::Rot3 world_R_body = world_R_odom * odom_ref.rotation();
    const gtsam::Point3 world_t_body = antennaEnuToBodyEnu(antenna_enu, world_R_body, config_.initial_lever_arm);

    // Compute world_T_odom
    const gtsam::Point3 world_t_odom_adjusted = world_t_body - world_R_odom.rotate(odom_t_body);
    const gtsam::Pose3 world_T_odom(world_R_odom, world_t_odom_adjusted);
    const gtsam::Pose3 x0 = world_T_odom * odom_ref;
    initial_.insert(X(0), x0);

    auto gauge_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1, 1, config_.init_yaw_prior_sigma, 1e3, 1e3, 1e3).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3(world_R_odom, gtsam::Point3()), gauge_noise));

    // Also add a GPSFactor to anchor position
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
