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

#include <memory>
#include <mutex>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>

#include "fusion/fusion_base.hpp"
#include "fusion/odom_buffer.hpp"
#include "fusion/gps_buffer.hpp"
#include "fusion/yaw_init.hpp"
#include "common/data_types.hpp"
#include "common/logger.hpp"
#include "common/parameter.hpp"

namespace odin_rtk_fusion {

using gtsam::symbol_shorthand::X;

// ============================================================================
// GraphFusion: ISAM2 factor graph fusion (odom + GPS + heading factors)
// ============================================================================

class GraphFusion final : public FusionBase {
public:
    using Config = FusionAlgorithmParams;

    // ------------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------------
    GraphFusion() : GraphFusion(Config()) {}
    explicit GraphFusion(const Config& config);

    // ------------------------------------------------------------------------
    // Public Interface (FusionBase overrides)
    // ------------------------------------------------------------------------
    void addOdom(const OdomData& odom, bool is_high_freq = false) override;
    void addGPS(const GPSData& gps) override;
    bool isInitialized() const override;
    Eigen::Matrix4d getWorldToOdom() const override;

private:
    // ------------------------------------------------------------------------
    // Internal Types
    // ------------------------------------------------------------------------
    enum class State {
        kInitializing,
        kRunning,
    };

    // ------------------------------------------------------------------------
    // Initialization
    // ------------------------------------------------------------------------
    void initializeWithFirstGps(const YawInitResult& init_result);

    // ------------------------------------------------------------------------
    // Factor Graph Operations
    // ------------------------------------------------------------------------
    void addOdomFactor(int from_id, int to_id, const gtsam::Pose3& from_pose, const gtsam::Pose3& to_pose);
    void addGpsFactor(int node_id, const gtsam::Pose3& odom_pose, const Eigen::Vector3d& enu, int quality, double hdop);
    void optimize();
    void marginalize();

    // ------------------------------------------------------------------------
    // Member Variables
    // ------------------------------------------------------------------------

    // Thread safety
    mutable std::mutex mutex_;

    // Configuration
    Config config_;

    // GTSAM components
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;

    // State machine
    State state_ = State::kInitializing;

    // Data buffers
    OdomBuffer odom_buffer_;           // Odom buffer (shared by init and runtime)
    GpsBuffer gps_buffer_;             // GPS buffer (used only during init)

    // Node management
    int node_count_ = 0;

    // Odom state
    bool have_odom_ = false;
    double last_node_timestamp_ = 0.0;  // Timestamp of the last node
    gtsam::Pose3 last_node_odom_pose_;  // Odom pose of the last node
    gtsam::Pose3 last_key_world_pose_;

    // World-to-odom transform (raw from ISAM2, updated at GPS rate)
    Eigen::Matrix4d world_T_odom_ = Eigen::Matrix4d::Identity();

    // Init alignment
    double init_yaw_offset_ = 0.0;
    double gps_time_offset_ = 0.0;  // GPS time compensation (added to GPS timestamp)

    // GPS heading estimation (from position differences)
    Eigen::Vector3d last_gps_enu_{0, 0, 0};
    bool have_last_gps_ = false;

    // GPS heading state (used in addGpsFactor for heading factor)
    double last_gps_heading_ = 0.0;
    double last_odom_yaw_at_gps_ = 0.0;
    bool have_gps_heading_ = false;

    // Last GPS body z for wTo z anchoring (prevent z drift in output)
    double last_gps_body_z_ = 0.0;

    // Marginalization
    int last_marginalized_id_ = -1;
};

} // namespace odin_rtk_fusion
