/*
 * NVIDIA contributions: line-search trajectory-filter lifecycle and
 * active-contact count interfaces; interfaces for frame-start active-set
 * reuse, candidate reuse within a detection skin, side-stream TOI filtering,
 * read-back buffers, the count read-back behind prelaunched work and the
 * detection hook before the candidate-count read-back.
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
#include <cuda_tool/cuda_tool.h>
#include <functional>

namespace uipc::backend::cuda
{
class TrajectoryFilter;
class GlobalContactManager;
class GlobalVertexManager;
class ContactExporterManager;

class GlobalTrajectoryFilter final : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    ~GlobalTrajectoryFilter() override;

    class Impl;

    class FilterTOIInfo
    {
      public:
        Float                     alpha() const noexcept { return m_alpha; }
        cuda_tool::VarView<Float> toi() const noexcept { return m_toi; }
        // Start positions of the swept step alpha * displacements.
        cuda_tool::CBufferView<Vector3> positions() const noexcept
        {
            return m_positions;
        }
        // Every launch of the filter's TOI work goes to this stream.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class GlobalTrajectoryFilter;
        Float                           m_alpha = 0.0;
        cuda_tool::VarView<Float>       m_toi;
        cuda_tool::CBufferView<Vector3> m_positions;
        cudaStream_t                    m_stream = cuda_tool::default_stream();
    };

    class DetectInfo
    {
      public:
        Float alpha() const noexcept { return m_alpha; }
        bool  reuse_bvh_topology() const noexcept
        {
            return m_reuse_bvh_topology;
        }
        // Extra pair distance (>= 0) added to every trajectory test, so that
        // later Newton iterations can reuse the candidates (see
        // reuse_candidates).
        Float skin() const noexcept { return m_skin; }
        // To be called by a filter right before it waits for the read-back of
        // its candidate counts, once its candidate lists are enqueued on the
        // legacy stream (possibly more than once per detect); null or empty:
        // nothing to call.
        const std::function<void()>* before_read_back() const noexcept
        {
            return m_before_read_back;
        }

      private:
        friend class GlobalTrajectoryFilter;
        Float                        m_alpha              = 0.0;
        bool                         m_reuse_bvh_topology = false;
        Float                        m_skin               = 0.0;
        const std::function<void()>* m_before_read_back   = nullptr;
    };

    class FilterActiveInfo
    {
      public:
        FilterActiveInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }
        // Every launch of a two-phase filter_active goes to this stream (a
        // side stream for the filters that run beside, see runs_beside()).
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class GlobalTrajectoryFilter;
        Impl*        m_impl;
        cudaStream_t m_stream = cuda_tool::default_stream();
    };

    class LabelActiveVerticesInfo
    {
      public:
        LabelActiveVerticesInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }
        cuda_tool::BufferView<IndexT> vert_is_active() const noexcept;

      private:
        friend class GlobalTrajectoryFilter;
        Impl* m_impl;
    };

    class RecordFrictionCandidatesInfo
    {
      public:
    };

    class Impl
    {
      public:
        void  init();
        Float filter_toi(Float alpha);
        Float candidate_skin();
        void  launch_filter_toi(Float                           alpha,
                                cuda_tool::CBufferView<Vector3> positions,
                                cudaStream_t                    stream);
        Float finish_filter_toi();

        SimSystemSlotCollection<TrajectoryFilter> filters;
        SimSystemSlot<GlobalContactManager>       global_contact_manager;
        SimSystemSlot<GlobalVertexManager>        global_vertex_manager;
        bool                                      friction_enabled = false;
        bool should_discard_friction_candidates                    = false;

        // Frame-start active-set reuse: the vertex positions at which the
        // last line-search trial of a completed Newton loop filtered the
        // active set (record_active_set), consumed by the next frame.
        bool                             active_set_recorded    = false;
        bool                             check_active_set_reuse = false;
        cuda_tool::DeviceBuffer<Vector3> active_set_positions;
        cuda_tool::DeviceVar<IndexT>     moved_vertices;

        // Newton-loop candidate reuse: whether every filter honors
        // DetectInfo::skin(), the skin of the held candidates (0: not
        // reusable) and the smallest per-vertex d_hat (< 0: not reduced yet).
        bool                        detect_skin_supported = false;
        Float                       held_skin             = 0.0;
        Float                       min_d_hat             = -1.0;
        cuda_tool::DeviceVar<Float> min_d_hat_reduced;


        // One TOI per filter, stored by the filter's TOI launches into a
        // read-back buffer (cuda_tool::ReadbackBuffer), read by
        // finish_filter_toi() after synchronizing toi_stream.
        cuda_tool::ReadbackBuffer<Float> tois;
        cudaStream_t                     toi_stream = nullptr;

        // filters whose two-phase filter_active was launched
        vector<char> filter_active_launched;
        // The streams of the filters that run beside the others
        // (TrajectoryFilter::runs_beside(), the half-plane filter) in
        // filter_active() and launch_filter_toi().
        cuda_tool::SideStream filter_active_side;
        cuda_tool::SideStream filter_toi_side;
        // Recorded after the filters' two-phase filter_active work when
        // filter_active() is given before_sync: `filtered` on the legacy
        // stream after the filters on it, `side_filtered` on the side stream
        // after the filters that run beside.
        cudaEvent_t filtered      = nullptr;
        cudaEvent_t side_filtered = nullptr;
    };

    template <std::derived_from<SimSystem> T>
    SimSystemSlot<T> find()
    {
        return m_impl.filters.find<T>();
    }

    void add_filter(TrajectoryFilter* filter);
    void require_discard_friction();

  private:
    virtual void do_build() override final;
    virtual void do_apply_recover(RecoverInfo& info) override final;

    friend class SimEngine;
    friend class ContactExporterManager;
    // called by SimEngine and ContactExporterManager; reusable (SimEngine's
    // Newton loop only) inflates a trajectory detection by the candidate skin;
    // before_read_back: see DetectInfo::before_read_back()
    void detect(Float                        alpha,
                bool                         reuse_bvh_topology = false,
                bool                         reusable           = false,
                const std::function<void()>& before_read_back   = {});
    // called by SimEngine and ContactExporterManager. With before_sync
    // (SimEngine's line-search trials only): once the two-phase work of the
    // filters on the legacy stream is enqueued, before_sync(filtered) is
    // called with an event recorded on the legacy stream after that work;
    // the filters that run beside are launched after it, and the read-back
    // of the active-set counts waits for `filtered` and their work only.
    // Work that before_sync enqueues on the legacy stream, or on a stream
    // that waits for `filtered`, may read the pending active set (the lists
    // at their capacity, the counts on the device; see
    // SimplexTrajectoryFilter::pending_active_set) and runs beside the
    // read-back and the filters that run beside, whose output it must not
    // read (the legacy stream waits for them after before_sync). Not called
    // when no filter launched a two-phase filter_active.
    void filter_active(const std::function<void(cudaEvent_t filtered)>& before_sync = {});
    // Newton-loop candidate reuse, only called by SimEngine: true when the
    // candidates of the last reusable detection serve a step over which every
    // vertex stays within axis_motion per axis of its detected swept segment.
    bool reuse_candidates(Float axis_motion) const;

    Float filter_toi(Float alpha);  // only called by SimEngine
    // The TOI of the step from the line-search start positions in two
    // halves, only called by SimEngine: launch_filter_toi() enqueues all of
    // its work, including the copy of the result to the host, on `stream`;
    // finish_filter_toi() waits for that stream and returns the TOI.
    void  launch_filter_toi(Float alpha, cudaStream_t stream);
    Float finish_filter_toi();
    void  record_friction_candidates();  // only called by SimEngine
    // Frame-start active-set reuse, only called by SimEngine
    bool reuse_active_set();   // at the frame start: true to skip the DCD pass
    void record_active_set();  // after a completed Newton loop
    void check_reused_active_set();  // UIPC_CHECK_ACTIVE_SET_REUSE=1
    friend class GlobalContactManager;
    void label_active_vertices();      // only called by GlobalContactManager
    void clear_friction_candidates();  // called by GlobalContactManager

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
