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

#include <eigen3/Eigen/Dense>
#include <string>

namespace odin_rtk_fusion {

/**
 * @brief 里程计数据
 */
struct OdomData {
    double timestamp = 0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    // 提取平移
    Eigen::Vector3d translation() const {
        return pose.block<3, 1>(0, 3);
    }

    // 提取旋转矩阵
    Eigen::Matrix3d rotation() const {
        return pose.block<3, 3>(0, 0);
    }
};

/**
 * @brief GPS/GGA数据
 */
struct GPSData {
    double timestamp = 0;
    double lat = 0;          // 纬度 (度)
    double lon = 0;          // 经度 (度)
    double alt = 0;          // 高度 (米)
    int quality = 0;         // 定位质量: 0=无效, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
    double hdop = 99.0;      // 水平精度因子
    int num_satellites = 0;  // 卫星数量
    bool valid = false;

    Eigen::Vector3d enu = Eigen::Vector3d::Zero();  // 转换后的ENU坐标
};

/**
 * @brief IMU数据 (预留)
 */
struct IMUData {
    double timestamp = 0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

} // namespace odin_rtk_fusion
