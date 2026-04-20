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
#pragma once

#include <mutex>
#include <eigen3/Eigen/Dense>

#include "fusion/fusion_base.hpp"
#include "fusion/odom_buffer.hpp"
#include "fusion/gps_buffer.hpp"
#include "fusion/yaw_init.hpp"
#include "common/data_types.hpp"
#include "common/logger.hpp"
#include "common/parameter.hpp"

namespace odin_rtk_fusion {

// ============================================================================
// AlignmentFusion: GPS calibrates wTo (yaw + offset), odom drives relative motion
// Windowed 2D Procrustes least-squares alignment. No factor graph.
// ============================================================================

class AlignmentFusion final : public FusionBase {
public:
    using Config = FusionAlgorithmParams;

    AlignmentFusion() : AlignmentFusion(Config()) {}
    explicit AlignmentFusion(const Config& config);

    void addOdom(const OdomData& odom, bool is_high_freq = false) override;
    void addGPS(const GPSData& gps) override;
    bool isInitialized() const override;
    Eigen::Matrix4d getWorldToOdom() const override;

private:
    enum class State { kInitializing, kRunning };

    // Configuration
    Config config_;

    // Thread safety
    mutable std::mutex mutex_;

    // State
    State state_ = State::kInitializing;

    // Odom state
    bool have_odom_ = false;
    OdomBuffer odom_buffer_;
    GpsBuffer gps_buffer_;
    double last_odom_timestamp_ = 0.0;
    Eigen::Matrix4d last_odom_pose_ = Eigen::Matrix4d::Identity();

    // wTo: world-to-odom transform (the output)
    Eigen::Matrix4d world_T_odom_ = Eigen::Matrix4d::Identity();

    // Init alignment
    double init_yaw_offset_ = 0.0;
    double gps_time_offset_ = 0.0;

    // Sliding window of (gps_enu, odom_xy, odom_z) pairs for least-squares wTo
    struct AlignPair {
        Eigen::Vector2d gps_xy;
        Eigen::Vector2d odom_xy;
        double gps_z;
        double odom_z;
    };
    std::vector<AlignPair> align_pairs_;
    static constexpr size_t kMaxAlignPairs = 50;

    void solveWTo();
};

} // namespace odin_rtk_fusion
