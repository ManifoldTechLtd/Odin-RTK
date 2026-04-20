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
#include <eigen3/Eigen/Dense>

namespace odin_rtk_fusion {

class GpsBuffer {
public:
    struct Sample {
        double timestamp;
        Eigen::Vector3d enu;
        int quality;
        double hdop;
    };

    explicit GpsBuffer(size_t max_size = 100) : max_size_(max_size) {}

    void push(double timestamp, const Eigen::Vector3d& enu, int quality, double hdop) {
        samples_.push_back(Sample{timestamp, enu, quality, hdop});
        while (samples_.size() > max_size_) {
            samples_.pop_front();
        }
    }

    void clear() { samples_.clear(); }

    bool empty() const { return samples_.empty(); }
    size_t size() const { return samples_.size(); }

    const Sample& front() const { return samples_.front(); }
    const Sample& back() const { return samples_.back(); }
    const Sample& operator[](size_t idx) const { return samples_[idx]; }

    // Iterator support
    auto begin() const { return samples_.begin(); }
    auto end() const { return samples_.end(); }

    void setMaxSize(size_t max_size) { max_size_ = max_size; }

private:
    std::deque<Sample> samples_;
    size_t max_size_;
};

}  // namespace odin_rtk_fusion
