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
#include "odom_buffer.hpp"

#include <algorithm>

namespace odin_rtk_fusion {

void OdomBuffer::push(double timestamp, const gtsam::Pose3& pose) {
    buffer_.push_back(Sample{timestamp, pose});
    
    while (buffer_.size() > max_size_) {
        buffer_.pop_front();
    }
}

void OdomBuffer::clear() {
    buffer_.clear();
}

OdomBuffer::QueryResult OdomBuffer::query(double timestamp) const {
    QueryResult result;
    
    if (buffer_.empty()) {
        return result;
    }
    
    // Edge case: timestamp out of range
    if (timestamp <= buffer_.front().timestamp) {
        result.found = true;
        result.before = buffer_.front();
        result.after = buffer_.front();
        result.alpha = 0.0;
        return result;
    }
    
    if (timestamp >= buffer_.back().timestamp) {
        result.found = true;
        result.before = buffer_.back();
        result.after = buffer_.back();
        result.alpha = 0.0;
        return result;
    }
    
    // Binary search for first element with timestamp >= target
    auto it = std::lower_bound(
        buffer_.begin(), buffer_.end(), timestamp,
        [](const Sample& s, double t) { return s.timestamp < t; }
    );
    
    if (it == buffer_.begin()) {
        result.found = true;
        result.before = *it;
        result.after = *it;
        result.alpha = 0.0;
        return result;
    }
    
    auto prev_it = std::prev(it);
    result.found = true;
    result.before = *prev_it;
    result.after = *it;
    
    const double dt = result.after.timestamp - result.before.timestamp;
    if (dt > 1e-9) {
        result.alpha = (timestamp - result.before.timestamp) / dt;
    } else {
        result.alpha = 0.0;
    }
    
    return result;
}

std::optional<OdomBuffer::Sample> OdomBuffer::nearest(double timestamp) const {
    if (buffer_.empty()) {
        return std::nullopt;
    }
    
    double best_dt = std::numeric_limits<double>::max();
    const Sample* best = nullptr;
    
    for (const auto& s : buffer_) {
        const double dt = std::abs(s.timestamp - timestamp);
        if (dt < best_dt) {
            best_dt = dt;
            best = &s;
        }
    }
    
    if (best) {
        return *best;
    }
    return std::nullopt;
}

double OdomBuffer::nearestTimeDiff(double timestamp) const {
    if (buffer_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    double best_dt = std::numeric_limits<double>::max();
    for (const auto& s : buffer_) {
        const double dt = std::abs(s.timestamp - timestamp);
        if (dt < best_dt) {
            best_dt = dt;
        }
    }
    return best_dt;
}

std::optional<gtsam::Pose3> OdomBuffer::interpolate(double timestamp) const {
    // Simplified implementation: return nearest frame
    // TODO: Implement proper pose interpolation (linear for translation + SLERP for rotation)
    auto sample = nearest(timestamp);
    if (sample) {
        return sample->pose;
    }
    return std::nullopt;
}

std::vector<OdomBuffer::Sample> OdomBuffer::range(double t_start, double t_end) const {
    std::vector<Sample> result;
    for (const auto& s : buffer_) {
        if (s.timestamp > t_start && s.timestamp <= t_end) {
            result.push_back(s);
        }
    }
    return result;
}

void OdomBuffer::popUntil(double t_end) {
    while (!buffer_.empty() && buffer_.front().timestamp <= t_end) {
        buffer_.pop_front();
    }
}

}  // namespace odin_rtk_fusion
