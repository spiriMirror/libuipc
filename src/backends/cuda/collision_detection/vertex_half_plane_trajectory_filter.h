/*
 * NVIDIA contributions: launch-stream, two-phase active-set filtering and
 * read-back interfaces.
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
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <cuda_tool/cuda_tool.h>
#include <implicit_geometry/half_plane.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class HalfPlaneVertexReporter;
class VertexHalfPlaneTrajectoryFilter : public TrajectoryFilter
{
  public:
    using TrajectoryFilter::TrajectoryFilter;

    class Impl;

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        Float                         d_hat() const noexcept;
        cuda_tool::CBufferView<Float> d_hats() const noexcept;

        IndexT half_plane_vertex_offset() const noexcept;
        cuda_tool::CBufferView<Vector3> plane_normals() const noexcept;
        cuda_tool::CBufferView<Vector3> plane_positions() const noexcept;

        cuda_tool::CBufferView<Vector3>  positions() const noexcept;
        cuda_tool::CBufferView<Float>    thicknesses() const noexcept;
        cuda_tool::CBufferView<IndexT>   contact_element_ids() const noexcept;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids() const noexcept;
        cuda_tool::CBuffer2DView<IndexT> contact_mask_tabular() const noexcept;
        cuda_tool::CBuffer2DView<IndexT> subscene_mask_tabular() const noexcept;
        cuda_tool::CBufferView<IndexT>   surf_vertices() const noexcept;

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        Impl* m_impl;
    };

    class DetectInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        Float                           alpha() const noexcept;
        cuda_tool::CBufferView<Vector3> displacements() const noexcept;

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        Float m_alpha;
    };

    class FilterActiveInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        /**
         * @brief Candidate vertex-half-plane pairs.
         */
        void PHs(cuda_tool::CBufferView<Vector2i> Ps) noexcept;
        // Every launch of the two-phase filter_active goes to this stream.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class VertexHalfPlaneTrajectoryFilter;
        cudaStream_t m_stream = cuda_tool::default_stream();
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
        friend class VertexHalfPlaneTrajectoryFilter;
        cuda_tool::VarView<Float>       m_toi;
        cuda_tool::CBufferView<Vector3> m_positions;
        cudaStream_t                    m_stream = cuda_tool::default_stream();
    };

    class BuildInfo
    {
      public:
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

        GlobalVertexManager* global_vertex_manager = nullptr;
        GlobalSimplicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager*    global_contact_manager     = nullptr;
        HalfPlane*               half_plane                 = nullptr;
        HalfPlaneVertexReporter* half_plane_vertex_reporter = nullptr;

        cuda_tool::CBufferView<Vector2i>  PHs;
        cuda_tool::DeviceBuffer<Vector2i> friction_PHs;
        cuda_tool::DeviceBuffer<Vector2i> recovered_PHs;

        Float reserve_ratio = 1.1;

        BufferDump dump_PHs;

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

    cuda_tool::CBufferView<Vector2i>         PHs() noexcept;
    cuda_tool::CBufferView<Vector2i>         friction_PHs() noexcept;
    virtual cuda_tool::CBufferView<Vector2i> candidate_PHs() const noexcept = 0;
    virtual cuda_tool::CBufferView<Float>    toi_PHs() const noexcept       = 0;

  protected:
    virtual void do_detect(DetectInfo& info)              = 0;
    virtual void do_filter_active(FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(FilterTOIInfo& info)       = 0;
    // Two-phase do_filter_active (see TrajectoryFilter): returning false from
    // the launch (the default) runs do_filter_active instead.
    virtual bool do_launch_filter_active(FilterActiveInfo& info);
    virtual void do_finish_filter_active(FilterActiveInfo& info);

    virtual void do_build(BuildInfo& info) {};
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    Impl         m_impl;
    virtual void do_build() override final;

    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) override final;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual bool do_launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) override final;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) override final;
    virtual void do_record_friction_candidates(
        GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info) override final;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) override final;
    virtual void do_clear_friction_candidates() override final;
};
}  // namespace uipc::backend::cuda
