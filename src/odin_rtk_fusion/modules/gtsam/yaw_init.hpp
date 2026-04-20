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
#pragma once

#include <optional>
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "odom_buffer.hpp"
#include "gps_buffer.hpp"

namespace odin_rtk_fusion {

struct YawInitResult {
    double yaw_offset;
    double time_offset;        // GPS-Odom 时间偏移（通过速度二范数对齐估计）
    double gps_timestamp;
    Eigen::Vector3d gps_enu;
    gtsam::Pose3 odom_ref;
};

struct YawInitConfig {
    double min_gps_displacement = 1.0;
    double min_odom_displacement = 1.0;
    size_t min_samples = 5;
    double outlier_thresh = 0.7;
    int stride = 3;
    // 时间偏移估计参数
    double time_offset_range = 0.5;     // 搜索范围 [-range, +range] 秒
    double time_offset_step = 0.01;     // 搜索步长 秒
};

// 计算初始 yaw 偏移
// 输入: odom 缓存, gps 缓存, 配置
// 输出: 计算结果 (yaw_offset, 参考 gps 时间戳, gps enu, odom 参考位姿)
std::optional<YawInitResult> computeInitYawOffset(
    const OdomBuffer& odom_buffer,
    const GpsBuffer& gps_buffer,
    const YawInitConfig& config);

}  // namespace odin_rtk_fusion
