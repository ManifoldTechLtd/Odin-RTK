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

#include <deque>
#include <optional>
#include <vector>
#include <cmath>

#include <gtsam/geometry/Pose3.h>

namespace odin_rtk_fusion {

// ============================================================================
// OdomBuffer - Odometry data buffer with timestamp query and interpolation
// ============================================================================

class OdomBuffer {
public:
    struct Sample {
        double timestamp;
        gtsam::Pose3 pose;
    };

    // Query result containing two surrounding frames (for future interpolation)
    struct QueryResult {
        bool found = false;
        Sample before;
        Sample after;
        double alpha = 0.0;  // Interpolation factor [0,1], t = before.t + alpha * (after.t - before.t)
    };

    // ------------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------------
    void setMaxSize(size_t max_size) { max_size_ = max_size; }
    size_t maxSize() const { return max_size_; }
    size_t size() const { return buffer_.size(); }
    bool empty() const { return buffer_.empty(); }

    // ------------------------------------------------------------------------
    // Data Input
    // ------------------------------------------------------------------------
    void push(double timestamp, const gtsam::Pose3& pose);
    void clear();

    // ------------------------------------------------------------------------
    // Query Interface
    // ------------------------------------------------------------------------
    
    // Query two surrounding frames at the given timestamp (for interpolation)
    QueryResult query(double timestamp) const;
    
    // Get nearest frame (simplified logic)
    std::optional<Sample> nearest(double timestamp) const;
    
    // Get time difference to nearest frame
    double nearestTimeDiff(double timestamp) const;
    
    // Interpolate pose (future extension, currently returns nearest frame)
    std::optional<gtsam::Pose3> interpolate(double timestamp) const;

    // Get all samples in time range (t_start, t_end]
    std::vector<Sample> range(double t_start, double t_end) const;
    
    // Remove data up to and including t_end
    void popUntil(double t_end);

    // ------------------------------------------------------------------------
    // Direct Access
    // ------------------------------------------------------------------------
    const Sample& front() const { return buffer_.front(); }
    const Sample& back() const { return buffer_.back(); }
    const Sample& operator[](size_t idx) const { return buffer_[idx]; }
    
    // Iterator support
    auto begin() const { return buffer_.begin(); }
    auto end() const { return buffer_.end(); }

private:
    std::deque<Sample> buffer_;
    size_t max_size_ = 500;
};

}  // namespace odin_rtk_fusion
