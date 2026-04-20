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
#include <cmath>
#include <string>
#include <utility>

namespace odin_rtk_fusion {

/**
 * @brief 经纬度到ENU局部坐标系的转换器
 *
 * 第一个有效GPS点自动设为ENU原点
 */
class ENUConverter {
public:
    ENUConverter() = default;

    ENUConverter(const ENUConverter&) = delete;
    ENUConverter& operator=(const ENUConverter&) = delete;

    ENUConverter(ENUConverter&& other) noexcept {
        moveFrom(std::move(other));
    }

    ENUConverter& operator=(ENUConverter&& other) noexcept {
        if (this != &other) {
            reset();
            moveFrom(std::move(other));
        }
        return *this;
    }

    ~ENUConverter() {
        reset();
    }

    bool init(const std::string& input_lla_crs) {
        input_lla_crs_ = input_lla_crs;
        if (input_lla_crs_.empty()) {
            return false;
        }

        if (input_lla_crs_ == "EPSG:4326") { // WGS84
            a_ = 6378137.0;
            f_ = 1.0 / 298.257223563;
        } else { // CGCS2000
            a_ = 6378137.0;
            f_ = 1.0 / 298.257222101;
        }
        e2_ = 2 * f_ - f_ * f_;
        return true;
    }

    /**
     * @brief 设置ENU坐标系原点
     * @param lat 纬度（度）
     * @param lon 经度（度）
     * @param alt 海拔高度（米）
     */
    void setOrigin(double lat, double lon, double alt) {
        if (input_lla_crs_.empty()) {
            input_lla_crs_ = "EPSG:4326";
            (void)init(input_lla_crs_);
        }

        lat0_rad_ = lat * M_PI / 180.0;
        lon0_rad_ = lon * M_PI / 180.0;
        alt0_m_ = alt;

        lat0_deg_ = lat;
        lon0_deg_ = lon;

        sin_lat0_ = std::sin(lat0_rad_);
        cos_lat0_ = std::cos(lat0_rad_);
        sin_lon0_ = std::sin(lon0_rad_);
        cos_lon0_ = std::cos(lon0_rad_);

        const double N0 = a_ / std::sqrt(1.0 - e2_ * sin_lat0_ * sin_lat0_);
        ecef0_x_ = (N0 + alt0_m_) * cos_lat0_ * cos_lon0_;
        ecef0_y_ = (N0 + alt0_m_) * cos_lat0_ * sin_lon0_;
        ecef0_z_ = (N0 * (1 - e2_) + alt0_m_) * sin_lat0_;

        initialized_ = true;
    }

    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief 将经纬度转换为ENU坐标
     * @param lat 纬度（度）
     * @param lon 经度（度）
     * @param alt 海拔高度（米）
     * @return ENU坐标 (East, North, Up)
     */
    Eigen::Vector3d toENU(double lat, double lon, double alt) const {
        if (!initialized_) {
            return Eigen::Vector3d::Zero();
        }

        const double lat_rad = lat * M_PI / 180.0;
        const double lon_rad = lon * M_PI / 180.0;

        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        const double N = a_ / std::sqrt(1.0 - e2_ * sin_lat * sin_lat);
        const double ecef_x = (N + alt) * cos_lat * cos_lon;
        const double ecef_y = (N + alt) * cos_lat * sin_lon;
        const double ecef_z = (N * (1 - e2_) + alt) * sin_lat;

        const double dx = ecef_x - ecef0_x_;
        const double dy = ecef_y - ecef0_y_;
        const double dz = ecef_z - ecef0_z_;

        const double e = -sin_lon0_ * dx + cos_lon0_ * dy;
        const double n = -sin_lat0_ * cos_lon0_ * dx - sin_lat0_ * sin_lon0_ * dy + cos_lat0_ * dz;
        const double u = cos_lat0_ * cos_lon0_ * dx + cos_lat0_ * sin_lon0_ * dy + sin_lat0_ * dz;

        return Eigen::Vector3d(e, n, u);
    }

    /**
     * @brief 将ENU坐标转换回经纬度
     * @param enu ENU坐标 (East, North, Up)
     * @param lat 输出纬度（度）
     * @param lon 输出经度（度）
     * @param alt 输出海拔高度（米）
     */
    void toLLA(const Eigen::Vector3d& enu, double& lat, double& lon, double& alt) const {
        if (!initialized_) {
            lat = lon = alt = 0;
            return;
        }

        const double e = enu.x();
        const double n = enu.y();
        const double u = enu.z();

        const double dx = -sin_lon0_ * e - sin_lat0_ * cos_lon0_ * n + cos_lat0_ * cos_lon0_ * u;
        const double dy = cos_lon0_ * e - sin_lat0_ * sin_lon0_ * n + cos_lat0_ * sin_lon0_ * u;
        const double dz = cos_lat0_ * n + sin_lat0_ * u;

        const double ecef_x = ecef0_x_ + dx;
        const double ecef_y = ecef0_y_ + dy;
        const double ecef_z = ecef0_z_ + dz;

        const double p = std::sqrt(ecef_x * ecef_x + ecef_y * ecef_y);
        lon = std::atan2(ecef_y, ecef_x);

        double lat_iter = std::atan2(ecef_z, p * (1 - e2_));
        for (int i = 0; i < 5; ++i) {
            const double sin_lat_iter = std::sin(lat_iter);
            const double N = a_ / std::sqrt(1.0 - e2_ * sin_lat_iter * sin_lat_iter);
            lat_iter = std::atan2(ecef_z + e2_ * N * sin_lat_iter, p);
        }
        lat = lat_iter;

        const double sin_lat_final = std::sin(lat);
        const double N_final = a_ / std::sqrt(1.0 - e2_ * sin_lat_final * sin_lat_final);
        alt = p / std::cos(lat) - N_final;

        lat = lat * 180.0 / M_PI;
        lon = lon * 180.0 / M_PI;
    }

    /**
     * @brief 获取原点的经纬度
     */
    void getOrigin(double& lat, double& lon, double& alt) const {
        lat = lat0_deg_;
        lon = lon0_deg_;
        alt = alt0_m_;
    }

private:
    void reset() {
        input_lla_crs_.clear();
        lat0_deg_ = 0.0;
        lon0_deg_ = 0.0;
        alt0_m_ = 0.0;

        lat0_rad_ = 0.0;
        lon0_rad_ = 0.0;

        sin_lat0_ = 0.0;
        cos_lat0_ = 1.0;
        sin_lon0_ = 0.0;
        cos_lon0_ = 1.0;

        ecef0_x_ = 0.0;
        ecef0_y_ = 0.0;
        ecef0_z_ = 0.0;

        a_ = 6378137.0;
        f_ = 1.0 / 298.257223563;
        e2_ = 2 * f_ - f_ * f_;

        initialized_ = false;
    }

    void moveFrom(ENUConverter&& other) {
        input_lla_crs_ = std::move(other.input_lla_crs_);
        lat0_deg_ = other.lat0_deg_;
        lon0_deg_ = other.lon0_deg_;
        alt0_m_ = other.alt0_m_;
        lat0_rad_ = other.lat0_rad_;
        lon0_rad_ = other.lon0_rad_;
        sin_lat0_ = other.sin_lat0_;
        cos_lat0_ = other.cos_lat0_;
        sin_lon0_ = other.sin_lon0_;
        cos_lon0_ = other.cos_lon0_;
        ecef0_x_ = other.ecef0_x_;
        ecef0_y_ = other.ecef0_y_;
        ecef0_z_ = other.ecef0_z_;
        a_ = other.a_;
        f_ = other.f_;
        e2_ = other.e2_;
        initialized_ = other.initialized_;

        other.initialized_ = false;
    }

private:
    std::string input_lla_crs_ = "EPSG:4326";

    double lat0_deg_ = 0.0;
    double lon0_deg_ = 0.0;
    double alt0_m_ = 0.0;

    double lat0_rad_ = 0.0;
    double lon0_rad_ = 0.0;

    double sin_lat0_ = 0.0;
    double cos_lat0_ = 1.0;
    double sin_lon0_ = 0.0;
    double cos_lon0_ = 1.0;

    double ecef0_x_ = 0.0;
    double ecef0_y_ = 0.0;
    double ecef0_z_ = 0.0;

    double a_ = 6378137.0;
    double f_ = 1.0 / 298.257223563;
    double e2_ = 2 * f_ - f_ * f_;

    bool initialized_ = false;
};

} // namespace odin_rtk_fusion
