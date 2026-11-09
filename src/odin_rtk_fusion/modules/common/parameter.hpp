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

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>

#include <type_traits>
#include <string>

#include "data_types.hpp"

namespace odin_rtk_fusion {

struct FusionAlgorithmParams {
    // Noise parameters
    double odom_position_sigma = 0.05;   // Odom position noise base (m/m)
    double odom_rotation_sigma = 0.1;    // Odom rotation noise base (rad/rad)
    double gps_base_sigma = 0.5;         // GPS noise base (multiplied by HDOP)

    double max_gps_match_dt = 0.5;       // Max GPS-odom matching time diff (s)
    int max_nodes = 100;                 // Max nodes (marginalization threshold)
    bool disable_gps_time_offset_estimation = false;

    // Initialization parameters
    int init_gps_buffer_size = 30;
    double init_min_gps_displacement = 1.0;   // Min GPS displacement for init (m)
    double init_min_odom_displacement = 1.0;
    double init_yaw_prior_sigma = 0.3;        // Initial yaw prior noise (rad)
    Eigen::Vector3d initial_lever_arm = Eigen::Vector3d::Zero();

    static FusionAlgorithmParams LoadFromYamlFile(const std::string& yaml_file) {
        FusionAlgorithmParams p;
        if (yaml_file.empty()) {
            return p;
        }

        const YAML::Node root = YAML::LoadFile(yaml_file);

        auto readScalar = [&root](const char* key, auto& out) {
            const YAML::Node n = root[key];
            if (n) {
                out = n.as<std::decay_t<decltype(out)>>();
            }
        };

        readScalar("odom_position_sigma", p.odom_position_sigma);
        readScalar("odom_rotation_sigma", p.odom_rotation_sigma);
        readScalar("gps_base_sigma", p.gps_base_sigma);

        readScalar("max_gps_match_dt", p.max_gps_match_dt);
        readScalar("max_nodes", p.max_nodes);
        readScalar("disable_gps_time_offset_estimation", p.disable_gps_time_offset_estimation);

        readScalar("init_gps_buffer_size", p.init_gps_buffer_size);
        readScalar("init_min_gps_displacement", p.init_min_gps_displacement);
        readScalar("init_min_odom_displacement", p.init_min_odom_displacement);
        readScalar("init_yaw_prior_sigma", p.init_yaw_prior_sigma);

        const YAML::Node lever = root["initial_lever_arm"];
        if (lever && lever.IsSequence() && lever.size() >= 3) {
            p.initial_lever_arm.x() = lever[0].as<double>();
            p.initial_lever_arm.y() = lever[1].as<double>();
            p.initial_lever_arm.z() = lever[2].as<double>();
        }

        return p;
    }
};

struct FusionNodeParams {
    FusionAlgorithmParams algorithm;

    // Algorithm selector: "graph" (ISAM2 factor graph) or "alignment" (windowed 2D Procrustes).
    std::string algorithm_type = "graph";

    std::string input_lla_crs = "EPSG:4326";

    bool output_project_enable = false;
    std::string output_project_target_crs;

    bool publish_tf = true;

    static FusionNodeParams LoadFromYamlFile(const std::string& yaml_file) {
        FusionNodeParams p;
        p.algorithm = FusionAlgorithmParams::LoadFromYamlFile(yaml_file);

        if (yaml_file.empty()) {
            return p;
        }

        const YAML::Node root = YAML::LoadFile(yaml_file);

        auto readScalar = [&root](const char* key, auto& out) {
            const YAML::Node n = root[key];
            if (n) {
                out = n.as<std::decay_t<decltype(out)>>();
            }
        };

        readScalar("algorithm_type", p.algorithm_type);
        readScalar("input_lla_crs", p.input_lla_crs);
        readScalar("output_project_enable", p.output_project_enable);
        readScalar("output_project_target_crs", p.output_project_target_crs);
        readScalar("publish_tf", p.publish_tf);
        return p;
    }
};

} // namespace odin_rtk_fusion
