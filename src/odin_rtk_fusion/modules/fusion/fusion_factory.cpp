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
#include "fusion/fusion_factory.hpp"

#include "common/logger.hpp"
#include "fusion/alignment/alignment_fusion.hpp"
#include "fusion/gtsam/graph_fusion.hpp"

namespace odin_rtk_fusion {

std::unique_ptr<FusionBase> createFusion(
    const std::string& algorithm_type,
    const FusionAlgorithmParams& config) {
    if (algorithm_type == "alignment") {
        LOG_INFO("[FusionFactory] Creating AlignmentFusion");
        return std::make_unique<AlignmentFusion>(config);
    }
    if (algorithm_type == "graph") {
        LOG_INFO("[FusionFactory] Creating GraphFusion");
        return std::make_unique<GraphFusion>(config);
    }
    LOG_WARN("[FusionFactory] Unknown algorithm_type='{}', falling back to 'graph'",
             algorithm_type);
    return std::make_unique<GraphFusion>(config);
}

}  // namespace odin_rtk_fusion
