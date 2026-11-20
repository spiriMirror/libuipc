/*
 * NVIDIA contributions: state and interfaces for the contact-tabular vertex
 * cover, concurrent tree chains, stored-pair judging and skins, BVH topology
 * retention, the fused detect prefix, the in-kernel TOI minimum and read-back
 * buffers; for the certified cover EE screens, the kept-tile scatter, the
 * detection hook before the candidate-count read-back and the EE TOI launch
 * beside the other TOI launches.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

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
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <array>

namespace uipc::backend::cuda
{
class InfoStacklessBVHSimplexTrajectoryFilter final : public SimplexTrajectoryFilter
{
  public:
    using SimplexTrajectoryFilter::SimplexTrajectoryFilter;
    ~InfoStacklessBVHSimplexTrajectoryFilter() override;

    class Impl
    {
      public:
        // concurrent: run the lbvh_T and lbvh_T_cover chains on their streams
        void detect(DetectInfo& info, bool concurrent);
        // filter_active in two phases: the launch enqueues the compaction and
        // the copy of its four counts, the finish sizes the active lists
        void launch_filter_active(FilterActiveInfo& info);
        void finish_filter_active(FilterActiveInfo& info);
        void filter_toi(FilterTOIInfo& info);

        /****************************************************
        *                   Broad Phase
        ****************************************************/

        cuda_tool::DeviceBuffer<AABB>   codim_point_aabbs;
        cuda_tool::DeviceBuffer<AABB>   point_aabbs;
        cuda_tool::DeviceBuffer<AABB>   edge_aabbs;
        cuda_tool::DeviceBuffer<AABB>   triangle_aabbs;
        cuda_tool::DeviceBuffer<IndexT> codim_point_bids;
        cuda_tool::DeviceBuffer<IndexT> point_bids;
        cuda_tool::DeviceBuffer<IndexT> edge_bids;
        cuda_tool::DeviceBuffer<IndexT> triangle_bids;
        cuda_tool::DeviceBuffer<IndexT> codim_point_cids;
        cuda_tool::DeviceBuffer<IndexT> point_cids;
        cuda_tool::DeviceBuffer<IndexT> edge_cids;
        cuda_tool::DeviceBuffer<IndexT> triangle_cids;

        using ThisBVH = InfoStacklessBVH;

        // CodimP count always less or equal to AllP count.
        ThisBVH              lbvh_CodimP;
        ThisBVH::QueryBuffer candidate_AllP_CodimP_pairs;

        // Used to detect CodimP-AllE, and AllE-AllE pairs.
        ThisBVH              lbvh_E;
        ThisBVH::QueryBuffer candidate_CodimP_AllE_pairs;
        ThisBVH::QueryBuffer candidate_AllE_AllE_pairs;

        // Used to detect AllP-AllT pairs.
        ThisBVH              lbvh_T;
        ThisBVH::QueryBuffer candidate_AllP_AllT_pairs;

        /****************************************************
        *               Contact-cover queries
        ****************************************************/

        // Every enabled contact pair has a contact element in a weighted vertex
        // cover S of the enabled contact-tabular graph, so EE and PT queries
        // can start from the primitives of S alone (subscene masks only
        // remove further pairs). Each family has its own S, weighted by its
        // query primitive counts.
        class CoverSplit
        {
          public:
            cuda_tool::DeviceBuffer<IndexT> is_query;  // per primitive: in S
            cuda_tool::DeviceBuffer<IndexT> ids;       // primitives in S first
            cuda_tool::DeviceBuffer<AABB>   aabbs;     // gathered in ids order
            cuda_tool::DeviceBuffer<AABB> grown_aabbs;  // stored pairs: grown copies
            cuda_tool::DeviceBuffer<IndexT> bids;
            cuda_tool::DeviceBuffer<IndexT> cids;
            // per primitive: its index in aabbs, bids and cids, or -1 (the
            // inverse of ids over the gathered range, kept with the split)
            cuda_tool::DeviceBuffer<IndexT> slots;
            SizeT                           count = 0;  // primitives in S
        };

        // Returns true when the S split of the primitives was refreshed.
        // with_gather == false: the caller's prefix writes the gathered lists
        // (only while the split stays, see cover_unchanged).
        bool update_cover(DetectInfo& info, bool with_gather = true);
        // The tabular, the primitive counts and a ready split are those of
        // the last detect: update_cover will neither recompute S nor split.
        bool cover_unchanged(DetectInfo& info) const;

        bool cover_EE = false;  // edges in S_E query lbvh_E
        bool cover_PT = false;  // points in S_P query lbvh_T, the others lbvh_T_cover
        CoverSplit cover_edges;
        CoverSplit cover_points;
        CoverSplit cover_triangles;  // split by S_P

        ThisBVH              lbvh_T_cover;  // triangles in S_P
        ThisBVH::QueryBuffer candidate_AllP_CoverT_pairs;

        // The lbvh_T and lbvh_T_cover chains of detect() run on these
        // streams, beside the lbvh_E chain on the default stream.
        cuda_tool::SideStream pt_stream;
        cuda_tool::SideStream cover_pt_stream;
        // Detects that may rebuild the topology (see TopologyRebuildPeriod).
        SizeT topology_requests = 0;

        // S is rebuilt when the tabular buffer (built once at init) or a
        // primitive count changes; a family keeps the original query when its
        // S holds >= 60% of the query primitives or the tabular is unusable.
        // S and the split depend only on the contact mask tabular and the
        // primitives' contact element ids, which this backend fixes at
        // World::init: GlobalContactManager::Impl::_build_contact_tabular,
        // called only from GlobalContactManager::Impl::init, is the one writer
        // of the mask tabular (adaptive kappa swaps only the coefficient
        // tabular); the vertex reporters write contact_element_ids in their
        // frame-0 init_attributes, and later attribute updates write positions
        // (the half-plane reporter rewrites the same ids); geometries created
        // after init stay pending, because the backend does not rebuild its
        // vertex or surface data (GlobalVertexManager::Impl::rebuild is not
        // implemented and no system handles event_rebuild_scene). A contact
        // tabular edited in the scene after World::init is therefore ignored
        // by the whole backend, not only here. A change that lets the mask or
        // the ids change at runtime must also refresh S and the split.
        const IndexT*        cover_tabular      = nullptr;
        SizeT                cover_tabular_size = 0;
        std::array<SizeT, 3> cover_sizes{};
        bool                 cover_EE_usable   = false;
        bool                 cover_PT_usable   = false;
        bool                 cover_split_ready = false;
        cuda_tool::DeviceBuffer<IndexT> edge_cover;  // per contact element: in S_E
        cuda_tool::DeviceBuffer<IndexT> point_cover;  // per contact element: in S_P
        cuda_tool::DeviceBuffer<IndexT> cover_scratch;
        cuda_tool::DeviceBuffer<IndexT> cover_counts;

        /****************************************************
        *                  Stored pairs
        ****************************************************/

        // A refresh runs every query on the boxes grown by a skin (per
        // primitive group and frame, see StoredSkinOverDHat in the .cu file)
        // and keeps the (query, leaf) pairs that pass the culls and the
        // position-free leaf checks. While every current box lies inside its
        // grown copy, these pairs hold every pair whose current boxes overlap,
        // so a detect judges only them, with the traversal's leaf tests. A box
        // outside its grown copy, a topology rebuild, a refreshed cover split
        // or a changed primitive count refreshes them (an escape refresh may
        // also raise a group's skin level, see stored_cover_thick).
        bool stored_enabled = false;  // tabular symmetric: culls are symmetric
        bool stored_valid   = false;
        std::array<SizeT, 4> stored_sizes{};  // codim points, points, edges, triangles
        cuda_tool::DeviceBuffer<AABB> grown_codim_point_aabbs;
        cuda_tool::DeviceBuffer<AABB> grown_point_aabbs;
        cuda_tool::DeviceBuffer<AABB> grown_edge_aabbs;
        cuda_tool::DeviceBuffer<AABB> grown_triangle_aabbs;
        ThisBVH::QueryBuffer          stored_AllP_CodimP_pairs;
        ThisBVH::QueryBuffer          stored_CodimP_AllE_pairs;
        ThisBVH::QueryBuffer          stored_AllE_AllE_pairs;
        ThisBVH::QueryBuffer          stored_AllP_AllT_pairs;
        ThisBVH::QueryBuffer          stored_AllP_CoverT_pairs;
        cuda_tool::DeviceBuffer<int> stored_flags;  // the five lists' judge flags
        // [0] a box left its grown copy; [1] a cover box, [2] another box left it
        // shrunk to its group's next lower skin level
        cuda_tool::DeviceBuffer<IndexT> stored_escaped;
        // Skin levels of the stored pairs with a cover: the cover primitives take
        // the thick skin instead of the uniform one, the others the uniform skin
        // instead of the thin one, for a frame after one in which some detect
        // found them outside the lower level (frame_*_left), and from a refresh
        // they cause on.
        bool stored_cover_thick   = false;
        bool stored_other_uniform = false;
        bool frame_cover_left     = false;
        bool frame_other_left     = false;
        cuda_tool::DeviceBuffer<IndexT> judge_tile_counts;  // kept pairs per 128-pair tile
        cuda_tool::DeviceBuffer<IndexT> judge_tile_offsets;  // their per-list prefix sums
        int judge_sm_count = 1;  // the scatter's and the EE passes' grids
        // the cover EE pairs that the screen leaves for the second certain test,
        // then those left for the predicate, and their counts (see
        // judge_screen_ee_kernel)
        cuda_tool::DeviceBuffer<int4> judge_ee_queue;
        cuda_tool::DeviceBuffer<int>  judge_ee_queue_counts;

        // FP32 inputs of the trajectory leaf tests' first rejection, per vertex
        // and detect: position and d_hat, step displacement and thickness.
        cuda_tool::DeviceBuffer<float4> f32_positions;
        cuda_tool::DeviceBuffer<float4> f32_steps;
        // The swept boxes' end points x + alpha dx in FP32, per vertex and
        // detect (the start points are f32_positions), for the prefix kernel.
        cuda_tool::DeviceBuffer<float4> f32_ends;

        // Four query counts (plus the cover-tree PT count and the three escape
        // flags) and four compacted PP/PE/PT/EE counts are each read back at
        // once. The judge scan and the compaction store theirs into read-back
        // buffers (cuda_tool::ReadbackBuffer); query_counts serves the direct
        // queries' count kernel and selected_counts DeviceSelect::If (lists
        // longer than CompactActiveListMaxSize), both followed by a copy.
        cuda_tool::DeviceBuffer<IndexT>   query_counts;
        cuda_tool::ReadbackBuffer<IndexT> judged_counts;
        cudaEvent_t judged = nullptr;  // after the judged counts' read-back copy
        // The TOI minimum's slot (toi_min.h): key, finished blocks.
        cuda_tool::DeviceBuffer<unsigned long long> toi_min_slot;
        // the EE TOI launch runs here, beside the other TOI launches
        cudaStream_t                      toi_side = nullptr;
        cudaEvent_t                       toi_fork = nullptr;
        cudaEvent_t                       toi_join = nullptr;
        cuda_tool::DeviceBuffer<IndexT>   selected_counts;
        cuda_tool::ReadbackBuffer<IndexT> selected_counts_readback;

        cuda_tool::DeviceBuffer<Vector4i> temp_PTs;
        cuda_tool::DeviceBuffer<Vector4i> temp_EEs;
        cuda_tool::DeviceBuffer<Vector3i> temp_PEs;
        cuda_tool::DeviceBuffer<Vector2i> temp_PPs;

        cuda_tool::DeviceBuffer<Vector4i> PTs;
        cuda_tool::DeviceBuffer<Vector4i> EEs;
        cuda_tool::DeviceBuffer<Vector3i> PEs;
        cuda_tool::DeviceBuffer<Vector2i> PPs;


        /****************************************************
        *                   CCD TOI
        ****************************************************/

        cuda_tool::DeviceBuffer<Float> tois;  // PP, PE, PT, EE
    };

    virtual cuda_tool::CBufferView<Vector2i> candidate_PTs() const noexcept override;
    virtual cuda_tool::CBufferView<Vector2i> candidate_EEs() const noexcept override;
    virtual cuda_tool::CBufferView<Float> toi_PTs() const noexcept override;
    virtual cuda_tool::CBufferView<Float> toi_EEs() const noexcept override;

  private:
    Impl m_impl;

    virtual void do_build(BuildInfo& info) override final;
    virtual void do_detect(DetectInfo& info) override final;
    virtual void do_filter_active(FilterActiveInfo& info) override final;
    virtual bool do_launch_filter_active(FilterActiveInfo& info) override final;
    virtual void do_finish_filter_active(FilterActiveInfo& info) override final;
    virtual void do_filter_toi(FilterTOIInfo& info) override final;
    virtual bool do_supports_detect_skin() const noexcept override final
    {
        return true;
    }
};
}  // namespace uipc::backend::cuda
