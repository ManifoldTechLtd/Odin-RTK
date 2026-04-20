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
#include "yaw_init.hpp"
#include "common/logger.hpp"

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace odin_rtk_fusion {

namespace {

struct YawSample {
    double yaw_offset;
    double weight;
};

double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

std::pair<bool, double> circularMean(const std::vector<YawSample>& samples) {
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    double sum_w = 0.0;
    for (const auto& s : samples) {
        if (!(s.weight > 0.0)) {
            continue;
        }
        sum_sin += s.weight * std::sin(s.yaw_offset);
        sum_cos += s.weight * std::cos(s.yaw_offset);
        sum_w += s.weight;
    }
    if (!(sum_w > 0.0)) {
        return {false, 0.0};
    }
    return {true, std::atan2(sum_sin, sum_cos)};
}

struct SpeedSample {
    double timestamp;
    double speed;  // Speed norm
};

// Compute speed sequence from GPS buffer
std::vector<SpeedSample> computeGpsSpeedSequence(const GpsBuffer& gps_buffer) {
    std::vector<SpeedSample> speeds;
    if (gps_buffer.size() < 2) {
        return speeds;
    }
    speeds.reserve(gps_buffer.size() - 1);
    for (size_t i = 1; i < gps_buffer.size(); ++i) {
        const auto& prev = gps_buffer[i - 1];
        const auto& curr = gps_buffer[i];
        const double dt = curr.timestamp - prev.timestamp;
        if (dt <= 0.0) {
            continue;
        }
        const Eigen::Vector3d delta = curr.enu - prev.enu;
        const double speed = delta.norm() / dt;
        const double mid_t = (prev.timestamp + curr.timestamp) * 0.5;
        speeds.push_back(SpeedSample{mid_t, speed});
    }
    return speeds;
}

// Compute speed sequence from Odom buffer
std::vector<SpeedSample> computeOdomSpeedSequence(const OdomBuffer& odom_buffer) {
    std::vector<SpeedSample> speeds;
    if (odom_buffer.size() < 2) {
        return speeds;
    }
    speeds.reserve(odom_buffer.size() - 1);
    for (size_t i = 1; i < odom_buffer.size(); ++i) {
        const auto& prev = odom_buffer[i - 1];
        const auto& curr = odom_buffer[i];
        const double dt = curr.timestamp - prev.timestamp;
        if (dt <= 0.0) {
            continue;
        }
        const gtsam::Point3 delta = curr.pose.translation() - prev.pose.translation();
        const double speed = delta.norm() / dt;
        const double mid_t = (prev.timestamp + curr.timestamp) * 0.5;
        speeds.push_back(SpeedSample{mid_t, speed});
    }
    return speeds;
}

// Interpolate speed at given time from odom speed sequence
double interpolateSpeed(const std::vector<SpeedSample>& speeds, double t) {
    if (speeds.empty()) {
        return 0.0;
    }
    if (t <= speeds.front().timestamp) {
        return speeds.front().speed;
    }
    if (t >= speeds.back().timestamp) {
        return speeds.back().speed;
    }
    // Binary search
    auto it = std::lower_bound(speeds.begin(), speeds.end(), t,
        [](const SpeedSample& s, double ts) { return s.timestamp < ts; });
    if (it == speeds.begin()) {
        return it->speed;
    }
    auto prev = std::prev(it);
    const double alpha = (t - prev->timestamp) / (it->timestamp - prev->timestamp);
    return prev->speed + alpha * (it->speed - prev->speed);
}

// Compute speed difference cost for a given time offset
double computeSpeedCost(const std::vector<SpeedSample>& gps_speeds,
                        const std::vector<SpeedSample>& odom_speeds,
                        double time_offset) {
    if (gps_speeds.empty() || odom_speeds.empty()) {
        return std::numeric_limits<double>::max();
    }
    double sum_sq_diff = 0.0;
    int count = 0;
    for (const auto& gs : gps_speeds) {
        const double odom_t = gs.timestamp + time_offset;
        const double odom_speed = interpolateSpeed(odom_speeds, odom_t);
        const double diff = gs.speed - odom_speed;
        sum_sq_diff += diff * diff;
        ++count;
    }
    return count > 0 ? sum_sq_diff / count : std::numeric_limits<double>::max();
}

// Estimate time offset (minimize speed norm difference)
double estimateTimeOffset(const OdomBuffer& odom_buffer,
                          const GpsBuffer& gps_buffer,
                          double range, double step) {
    const auto gps_speeds = computeGpsSpeedSequence(gps_buffer);
    const auto odom_speeds = computeOdomSpeedSequence(odom_buffer);
    
    if (gps_speeds.size() < 2 || odom_speeds.size() < 2) {
        return 0.0;
    }
    
    // Output GPS speed sequence for debugging
    for (const auto& s : gps_speeds) {
        LOG_INFO("[SPEED_LOG] GPS {:.6f} {:.4f}", s.timestamp, s.speed);
    }
    // Output Odom speed sequence for debugging
    for (const auto& s : odom_speeds) {
        LOG_INFO("[SPEED_LOG] ODOM {:.6f} {:.4f}", s.timestamp, s.speed);
    }
    
    double best_offset = 0.0;
    double best_cost = std::numeric_limits<double>::max();
    
    for (double offset = -range; offset <= range; offset += step) {
        const double cost = computeSpeedCost(gps_speeds, odom_speeds, offset);
        if (cost < best_cost) {
            best_cost = cost;
            best_offset = offset;
        }
    }
    
    // Output best offset
    LOG_INFO("[SPEED_LOG] BEST_OFFSET {:.6f}", best_offset);
    
    return best_offset;
}

}  // namespace

std::optional<YawInitResult> computeInitYawOffset(
    const OdomBuffer& odom_buffer,
    const GpsBuffer& gps_buffer,
    const YawInitConfig& config) {
    
    if (gps_buffer.size() < config.min_samples || odom_buffer.empty()) {
        return std::nullopt;
    }

    const auto& last = gps_buffer.back();
    auto odom_last_opt = odom_buffer.nearest(last.timestamp);
    if (!odom_last_opt) {
        return std::nullopt;
    }
    const gtsam::Pose3 odom_last = odom_last_opt->pose;

    std::vector<YawSample> samples;
    samples.reserve(gps_buffer.size());

    for (size_t i = 0; i + 1 < gps_buffer.size(); i += static_cast<size_t>(config.stride)) {
        const auto& gi = gps_buffer[i];
        const Eigen::Vector2d gps_delta_xy(last.enu.x() - gi.enu.x(), last.enu.y() - gi.enu.y());
        const double gps_dist = gps_delta_xy.norm();
        if (gps_dist < config.min_gps_displacement) {
            continue;
        }

        auto odom_i_opt = odom_buffer.nearest(gi.timestamp);
        if (!odom_i_opt) {
            continue;
        }
        const gtsam::Pose3 odom_i = odom_i_opt->pose;

        const gtsam::Pose3 odom_delta = odom_i.inverse() * odom_last;
        const gtsam::Point3 odom_dp = odom_delta.translation();
        const Eigen::Vector2d odom_delta_xy(odom_dp.x(), odom_dp.y());
        const double odom_dist = odom_delta_xy.norm();
        if (odom_dist < config.min_odom_displacement) {
            continue;
        }

        const double yaw_gps = std::atan2(gps_delta_xy.y(), gps_delta_xy.x());
        const double yaw_odom = std::atan2(odom_delta_xy.y(), odom_delta_xy.x());
        const double yaw_offset = wrapAngle(yaw_gps - yaw_odom);

        const double w = std::min(gps_dist, odom_dist);
        samples.push_back(YawSample{yaw_offset, w});
    }

    if (samples.size() < config.min_samples) {
        return std::nullopt;
    }

    const auto mean0 = circularMean(samples);
    if (!mean0.first) {
        return std::nullopt;
    }

    std::vector<YawSample> inliers;
    inliers.reserve(samples.size());
    for (const auto& s : samples) {
        const double err = std::abs(wrapAngle(s.yaw_offset - mean0.second));
        if (err <= config.outlier_thresh) {
            inliers.push_back(s);
        }
    }

    const auto mean1 = circularMean(inliers);
    if (!mean1.first) {
        return std::nullopt;
    }

    const auto& first = gps_buffer.front();
    auto odom_first_opt = odom_buffer.nearest(first.timestamp);
    if (!odom_first_opt) {
        return std::nullopt;
    }

    // Estimate time offset (via speed norm alignment)
    const double time_offset = estimateTimeOffset(
        odom_buffer, gps_buffer,
        config.time_offset_range, config.time_offset_step);

    // time_offset meaning: GPS timestamp + time_offset = corresponding odom timestamp
    // time_offset < 0: GPS clock is slower than odom (odom curve leads on time axis)
    // time_offset > 0: odom clock is faster than GPS (GPS curve leads on time axis)
    LOG_INFO("[YawInit] yaw_offset: {:.4f} rad ({:.2f} deg), time_offset: {:.4f} s (GPS clock {} than odom by {:.4f}s)",
             mean1.second, mean1.second * 180.0 / M_PI, time_offset,
             time_offset <= 0 ? "slower" : "faster", std::abs(time_offset));

    return YawInitResult{
        mean1.second,
        time_offset,
        first.timestamp,
        first.enu,
        odom_first_opt->pose
    };
}

}  // namespace odin_rtk_fusion
