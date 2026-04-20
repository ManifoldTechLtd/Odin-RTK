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

#include <memory>
#include <string>

#include "common/parameter.hpp"
#include "fusion/fusion_base.hpp"

namespace odin_rtk_fusion {

// ============================================================================
// Fusion algorithm factory
// ============================================================================
//
// Build a fusion implementation by algorithm name.
//
// Supported values for `algorithm_type`:
//   - "graph"     : GraphFusion (ISAM2 factor graph, full optimization)
//   - "alignment" : AlignmentFusion (windowed 2D Procrustes closed-form alignment)
//
// Unknown or empty names fall back to "graph" with a warning.
// ============================================================================
std::unique_ptr<FusionBase> createFusion(
    const std::string& algorithm_type,
    const FusionAlgorithmParams& config);

}  // namespace odin_rtk_fusion
