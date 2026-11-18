/*
 * NVIDIA contributions: active-contact counting and line-search trajectory
 * filtering interfaces; detection-skin, launch-stream TOI, friction-candidate
 * version, two-phase active-set filtering and pending active-set interfaces;
 * the detection hook before the candidate-count read-back.
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
#include <collision_detection/trajectory_filter.h>
#include <collision_detection/global_trajectory_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/global_body_manager.h>
#include <contact_system/global_contact_manager.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class SimplexTrajectoryFilter : public TrajectoryFilter
{
  public:
    using TrajectoryFilter::TrajectoryFilter;

    class Impl;

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        Float d_hat() const noexcept;
        Float toi_safety_margin() const noexcept;

        // Vertex Attributes

        /**
         * @brief Vertex Id to Body Id mapping.
         */
        cuda_tool::CBufferView<Float>    d_hats() const noexcept;
        cuda_tool::CBufferView<IndexT>   v2b() const noexcept;
        cuda_tool::CBufferView<Vector3>  positions() const noexcept;
        cuda_tool::CBufferView<Vector3>  rest_positions() const noexcept;
        cuda_tool::CBufferView<Float>    thicknesses() const noexcept;
        cuda_tool::CBufferView<IndexT>   dimensions() const noexcept;
        cuda_tool::CBufferView<IndexT>   contact_element_ids() const noexcept;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids() const noexcept;
        cuda_tool::CBuffer2DView<IndexT> contact_mask_tabular() const noexcept;
        cuda_tool::CBuffer2DView<IndexT> subscene_mask_tabular() const noexcept;
        // Body Attributes

        /**
         * @brief Tell if the body needs self-collision
         */
        cuda_tool::CBufferView<IndexT> body_self_collision() const noexcept;

        // Topologies

        cuda_tool::CBufferView<IndexT>   codim_vertices() const noexcept;
        cuda_tool::CBufferView<IndexT>   surf_vertices() const noexcept;
        cuda_tool::CBufferView<Vector2i> surf_edges() const noexcept;
        cuda_tool::CBufferView<Vector3i> surf_triangles() const noexcept;

      protected:
        friend class SimplexTrajectoryFilter;
        Impl* m_impl = nullptr;
    };

    class DetectInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        Float alpha() const noexcept { return m_alpha; }
        bool  reuse_bvh_topology() const noexcept
        {
            return m_reuse_bvh_topology;
        }
        // See GlobalTrajectoryFilter::DetectInfo::skin.
        Float skin() const noexcept { return m_skin; }
        // See GlobalTrajectoryFilter::DetectInfo::before_read_back (may be
        // empty).
        void before_read_back() const
        {
            if(m_before_read_back && *m_before_read_back)
                (*m_before_read_back)();
        }

        cuda_tool::CBufferView<Vector3> displacements() const noexcept;

      private:
        friend class SimplexTrajectoryFilter;
        Float                        m_alpha              = 0.0;
        bool                         m_reuse_bvh_topology = false;
        Float                        m_skin               = 0.0;
        const std::function<void()>* m_before_read_back   = nullptr;
    };

    // The active set of a two-phase filter_active between its launch and
    // its finish: every list at its capacity, of which the leading pairs, as
    // many as the count stored at the list's device address, are active.
    // Null counts: none published.
    class PendingActiveSet
    {
      public:
        cuda_tool::CBufferView<Vector4i> PTs;
        cuda_tool::CBufferView<Vector4i> EEs;
        cuda_tool::CBufferView<Vector3i> PEs;
        cuda_tool::CBufferView<Vector2i> PPs;
        const IndexT*                    PT_count = nullptr;
        const IndexT*                    EE_count = nullptr;
        const IndexT*                    PE_count = nullptr;
        const IndexT*                    PP_count = nullptr;
    };

    class FilterActiveInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        /**
         * @brief Optional, in the launch of a two-phase filter_active: the
         * active set it enqueued (see pending_active_set()).
         */
        void pending(const PendingActiveSet& pending) noexcept;

        /**
         * @brief Candidate point-triangle pairs.
         */
        void PTs(cuda_tool::CBufferView<Vector4i> PTs) noexcept;
        /**
         * @brief Candidate edge-edge pairs.
         */
        void EEs(cuda_tool::CBufferView<Vector4i> EEs) noexcept;
        /**
         * @brief Candidate point-edge pairs.
         */
        void PEs(cuda_tool::CBufferView<Vector3i> PEs) noexcept;
        /**
         * @brief Candidate point-point pairs.
         */
        void PPs(cuda_tool::CBufferView<Vector2i> PPs) noexcept;
    };

    class FilterTOIInfo : public DetectInfo
    {
      public:
        using DetectInfo::DetectInfo;

        cuda_tool::VarView<Float> toi() noexcept;
        // Start positions of the swept step (not always the current ones).
        cuda_tool::CBufferView<Vector3> positions() const noexcept;
        // Every launch of the TOI work goes to this stream.
        cudaStream_t stream() const noexcept;

      private:
        friend class SimplexTrajectoryFilter;
        cuda_tool::VarView<Float>       m_toi;
        cuda_tool::CBufferView<Vector3> m_positions;
        cudaStream_t                    m_stream = cuda_tool::default_stream();
    };

    class Impl
    {
      public:
        void record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info);
        void label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info);
        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_simplicial_surface_manager;
        SimSystemSlot<GlobalContactManager> global_contact_manager;
        SimSystemSlot<GlobalBodyManager>    global_body_manager;
        Float                               toi_safety_margin = 0.1;

        cuda_tool::CBufferView<Vector4i> PTs;
        cuda_tool::CBufferView<Vector4i> EEs;
        cuda_tool::CBufferView<Vector3i> PEs;
        cuda_tool::CBufferView<Vector2i> PPs;
        PendingActiveSet                 pending;

        cuda_tool::DeviceBuffer<Vector4i> friction_PT;
        cuda_tool::DeviceBuffer<Vector4i> friction_EE;
        cuda_tool::DeviceBuffer<Vector3i> friction_PE;
        cuda_tool::DeviceBuffer<Vector2i> friction_PP;
        // Incremented whenever the friction candidates are recorded or cleared.
        SizeT friction_candidates_version = 0;

        cuda_tool::DeviceBuffer<Vector4i> recovered_PT;
        cuda_tool::DeviceBuffer<Vector4i> recovered_EE;
        cuda_tool::DeviceBuffer<Vector3i> recovered_PE;
        cuda_tool::DeviceBuffer<Vector2i> recovered_PP;

        Float reserve_ratio = 1.1;

        BufferDump dump_PTs;
        BufferDump dump_EEs;
        BufferDump dump_PEs;
        BufferDump dump_PPs;

        template <typename T>
        void loose_resize(cuda_tool::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve_discard(size * reserve_ratio);
            }
            buffer.resize_discard(size);
        }
    };

    cuda_tool::CBufferView<Vector4i> PTs() const noexcept;
    cuda_tool::CBufferView<Vector4i> EEs() const noexcept;
    cuda_tool::CBufferView<Vector3i> PEs() const noexcept;
    cuda_tool::CBufferView<Vector2i> PPs() const noexcept;
    // Published by the launch of a two-phase filter_active until its
    // finish (see GlobalTrajectoryFilter::filter_active's before_sync).
    const PendingActiveSet& pending_active_set() const noexcept;

    cuda_tool::CBufferView<Vector4i> friction_PTs() const noexcept;
    cuda_tool::CBufferView<Vector4i> friction_EEs() const noexcept;
    cuda_tool::CBufferView<Vector3i> friction_PEs() const noexcept;
    cuda_tool::CBufferView<Vector2i> friction_PPs() const noexcept;
    // Changes whenever the friction candidates (and the frame-start
    // positions they were recorded at) change.
    SizeT friction_candidates_version() const noexcept;

    virtual cuda_tool::CBufferView<Vector2i> candidate_PTs() const noexcept = 0;
    virtual cuda_tool::CBufferView<Vector2i> candidate_EEs() const noexcept = 0;
    virtual cuda_tool::CBufferView<Float>    toi_PTs() const noexcept       = 0;
    virtual cuda_tool::CBufferView<Float>    toi_EEs() const noexcept       = 0;

  protected:
    virtual void do_build(BuildInfo& info)                = 0;
    virtual void do_detect(DetectInfo& info)              = 0;
    virtual void do_filter_active(FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(FilterTOIInfo& info)       = 0;
    // Two-phase do_filter_active (see TrajectoryFilter): returning false from
    // the launch (the default) runs do_filter_active instead.
    virtual bool do_launch_filter_active(FilterActiveInfo& info);
    virtual void do_finish_filter_active(FilterActiveInfo& info);
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    friend class GlobalDCDFilter;
    Impl m_impl;

    virtual void do_build() override final;

    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) override final;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual bool do_launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) override final;
    virtual void do_record_friction_candidates(
        GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info) override final;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) final override;
    virtual void do_clear_friction_candidates() override final;
};
}  // namespace uipc::backend::cuda
