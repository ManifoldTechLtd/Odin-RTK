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

#include <eigen3/Eigen/Dense>
#include <string>

namespace odin_rtk_fusion {

/**
 * @brief Odometry data
 */
struct OdomData {
    double timestamp = 0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    // Extract translation
    Eigen::Vector3d translation() const {
        return pose.block<3, 1>(0, 3);
    }

    // Extract rotation matrix
    Eigen::Matrix3d rotation() const {
        return pose.block<3, 3>(0, 0);
    }
};

/**
 * @brief GPS/GGA data
 */
struct GPSData {
    double timestamp = 0;
    double lat = 0;          // Latitude (degrees)
    double lon = 0;          // Longitude (degrees)
    double alt = 0;          // Altitude (meters)
    int quality = 0;         // Fix quality: 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
    double hdop = 99.0;      // Horizontal dilution of precision
    int num_satellites = 0;  // Number of satellites
    bool valid = false;

    Eigen::Vector3d enu = Eigen::Vector3d::Zero();  // Converted ENU coordinates
};

/**
 * @brief IMU data (reserved)
 */
struct IMUData {
    double timestamp = 0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

} // namespace odin_rtk_fusion
