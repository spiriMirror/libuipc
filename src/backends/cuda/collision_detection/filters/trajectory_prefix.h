/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <uipc/backend/macro.h>
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <cuda_runtime.h>

namespace uipc::backend::cuda
{
// The prefix of a trajectory detect in two launches: per vertex the FP32
// inputs of the leaf tests and the swept boxes' FP32 end points, then per
// primitive (one block range per family) the swept box with its bid and cid,
// its copy in the family's gathered cover lists and the stored pairs'
// containment check. It writes what InfoStacklessBVHSimplexTrajectoryFilter's
// separate kernels write (detect_k1-k4, cover_gather, f32_inputs, the escape
// flag fill and contained), bit for bit.

// One primitive family: points (vertices: codim or surface vertex ids), edges
// or triangles.
struct TrajectoryPrefixFamily
{
    const IndexT*   vertices  = nullptr;
    const Vector2i* edges     = nullptr;
    const Vector3i* triangles = nullptr;
    int             n         = 0;
    AABB*           aabbs     = nullptr;
    IndexT*         bids      = nullptr;
    IndexT*         cids      = nullptr;
    // Cover gather (null: none): per primitive its index in the gathered
    // lists, or -1.
    const IndexT* slots          = nullptr;
    AABB*         gathered_aabbs = nullptr;
    IndexT*       gathered_bids  = nullptr;
    IndexT*       gathered_cids  = nullptr;
    // Containment check against the grown copies (null: none), with the cover
    // split of the shrunk checks (null: none).
    const AABB*   grown    = nullptr;
    const IndexT* is_query = nullptr;
};

constexpr int TrajectoryPrefixFamilies = 4;  // codim points, points, edges, triangles

struct TrajectoryPrefix
{
    TrajectoryPrefixFamily family[TrajectoryPrefixFamilies];
    // per vertex
    const Vector3* positions     = nullptr;
    const Vector3* displacements = nullptr;
    const Float*   d_hats        = nullptr;
    const Float*   thicknesses   = nullptr;
    const IndexT*  v2bs          = nullptr;
    const IndexT*  contact_ids   = nullptr;
    int            vertices      = 0;
    Float          alpha         = 0;
    // per vertex outputs: position and d_hat, step displacement and
    // thickness (the leaf tests' FP32 inputs), and the swept boxes' end points
    float4* f32_positions = nullptr;
    float4* f32_steps     = nullptr;
    float4* f32_ends      = nullptr;
    // containment check: the escape flags (cleared first) and the shrinks to
    // the groups' next lower skin levels
    IndexT* escaped      = nullptr;
    float   cover_shrink = 0.0f;
    float   other_shrink = 0.0f;
};

// Launches the prefix on `stream`; the families' containment checks need
// `escaped`.
UIPC_BACKEND_API void launch_trajectory_prefix(const TrajectoryPrefix& prefix,
                                               cudaStream_t            stream);
}  // namespace uipc::backend::cuda
