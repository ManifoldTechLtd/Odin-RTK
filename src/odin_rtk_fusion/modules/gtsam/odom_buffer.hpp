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

#include <deque>
#include <optional>
#include <vector>
#include <cmath>

#include <gtsam/geometry/Pose3.h>

namespace odin_rtk_fusion {

// ============================================================================
// OdomBuffer - 里程计数据缓存，支持时间戳查询和插值
// ============================================================================

class OdomBuffer {
public:
    struct Sample {
        double timestamp;
        gtsam::Pose3 pose;
    };

    // 用于时间戳查询的结果，包含前后两帧信息（支持未来插值）
    struct QueryResult {
        bool found = false;
        Sample before;
        Sample after;
        double alpha = 0.0;  // 插值系数 [0,1], t = before.t + alpha * (after.t - before.t)
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
    
    // 查询指定时间戳的前后两帧（用于插值）
    QueryResult query(double timestamp) const;
    
    // 获取最近的一帧（当前简化逻辑）
    std::optional<Sample> nearest(double timestamp) const;
    
    // 获取最近帧的时间差
    double nearestTimeDiff(double timestamp) const;
    
    // 插值获取位姿（未来扩展，目前返回最近帧）
    std::optional<gtsam::Pose3> interpolate(double timestamp) const;

    // 获取时间范围内的所有样本 (t_start, t_end]
    std::vector<Sample> range(double t_start, double t_end) const;
    
    // 移除 t_end 及之前的数据
    void popUntil(double t_end);

    // ------------------------------------------------------------------------
    // Direct Access
    // ------------------------------------------------------------------------
    const Sample& front() const { return buffer_.front(); }
    const Sample& back() const { return buffer_.back(); }
    const Sample& operator[](size_t idx) const { return buffer_[idx]; }
    
    // 迭代器支持
    auto begin() const { return buffer_.begin(); }
    auto end() const { return buffer_.end(); }

private:
    std::deque<Sample> buffer_;
    size_t max_size_ = 500;
};

}  // namespace odin_rtk_fusion
