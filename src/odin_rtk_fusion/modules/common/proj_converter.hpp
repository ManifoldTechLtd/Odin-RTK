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

#include <proj.h>

#include <cmath>
#include <string>
#include <utility>

namespace odin_rtk_fusion {

class ProjConverter {
public:
    ProjConverter() = default;

    ProjConverter(const ProjConverter&) = delete;
    ProjConverter& operator=(const ProjConverter&) = delete;

    ProjConverter(ProjConverter&& other) noexcept {
        moveFrom(std::move(other));
    }

    ProjConverter& operator=(ProjConverter&& other) noexcept {
        if (this != &other) {
            reset();
            moveFrom(std::move(other));
        }
        return *this;
    }

    ~ProjConverter() {
        reset();
    }

    bool init(const std::string& target_crs) {
        return init("EPSG:4326", target_crs);
    }

    bool init(const std::string& source_crs, const std::string& target_crs) {
        reset();
        if (source_crs.empty() || target_crs.empty()) {
            return false;
        }

        ctx_ = proj_context_create();
        if (!ctx_) {
            return false;
        }

        PJ* raw = proj_create_crs_to_crs(ctx_, source_crs.c_str(), target_crs.c_str(), nullptr);
        if (!raw) {
            reset();
            return false;
        }

        transform_ = proj_normalize_for_visualization(ctx_, raw);
        proj_destroy(raw);

        if (!transform_) {
            reset();
            return false;
        }

        source_crs_ = source_crs;
        target_crs_ = target_crs;
        return true;
    }

    bool isInitialized() const {
        return ctx_ != nullptr && transform_ != nullptr;
    }

    const std::string& sourceCrs() const {
        return source_crs_;
    }

    const std::string& targetCrs() const {
        return target_crs_;
    }

    bool projectWgs84Deg(double lat_deg, double lon_deg, double alt_m, double& out_x, double& out_y, double& out_z) const {
        return projectDeg(lat_deg, lon_deg, alt_m, out_x, out_y, out_z);
    }

    bool projectDeg(double lat_deg, double lon_deg, double alt_m, double& out_x, double& out_y, double& out_z) const {
        if (!isInitialized()) {
            return false;
        }

        PJ_COORD in;
        in.lpzt.lam = lon_deg;
        in.lpzt.phi = lat_deg;
        in.lpzt.z = alt_m;
        in.lpzt.t = 0.0;

        PJ_COORD out = proj_trans(transform_, PJ_FWD, in);
        if (!std::isfinite(out.xyzt.x) || !std::isfinite(out.xyzt.y) || !std::isfinite(out.xyzt.z)) {
            return false;
        }

        out_x = out.xyzt.x;
        out_y = out.xyzt.y;
        out_z = out.xyzt.z;
        return true;
    }

private:
    void reset() {
        if (transform_) {
            proj_destroy(transform_);
            transform_ = nullptr;
        }
        if (ctx_) {
            proj_context_destroy(ctx_);
            ctx_ = nullptr;
        }
        source_crs_.clear();
        target_crs_.clear();
    }

    void moveFrom(ProjConverter&& other) {
        ctx_ = other.ctx_;
        transform_ = other.transform_;
        source_crs_ = std::move(other.source_crs_);
        target_crs_ = std::move(other.target_crs_);

        other.ctx_ = nullptr;
        other.transform_ = nullptr;
    }

private:
    PJ_CONTEXT* ctx_ = nullptr;
    PJ* transform_ = nullptr;
    std::string source_crs_;
    std::string target_crs_;
};

} // namespace odin_rtk_fusion
