/*
 * NVIDIA contributions: line-search trajectory-filter lifecycle and
 * active-contact count interfaces; interfaces for frame-start active-set
 * reuse, candidate reuse within a detection skin and two-phase active-set
 * filtering.
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

    class Impl;

    class FilterTOIInfo
    {
      public:
        Float                     alpha() const noexcept { return m_alpha; }
        cuda_tool::VarView<Float> toi() const noexcept { return m_toi; }

      private:
        friend class GlobalTrajectoryFilter;
        Float                     m_alpha = 0.0;
        cuda_tool::VarView<Float> m_toi;
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

      private:
        friend class GlobalTrajectoryFilter;
        Float m_alpha              = 0.0;
        bool  m_reuse_bvh_topology = false;
        Float m_skin               = 0.0;
    };

    class FilterActiveInfo
    {
      public:
        FilterActiveInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }


      private:
        friend class GlobalTrajectoryFilter;
        Impl* m_impl;
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


        cuda_tool::DeviceBuffer<Float> tois;
        vector<Float>                  h_tois;

        // filters whose two-phase filter_active was launched
        vector<char> filter_active_launched;
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
    // Newton loop only) inflates a trajectory detection by the candidate skin
    void detect(Float alpha, bool reuse_bvh_topology = false, bool reusable = false);
    void filter_active();  // called by SimEngine and ContactExporterManager
    // Newton-loop candidate reuse, only called by SimEngine: true when the
    // candidates of the last reusable detection serve a step over which every
    // vertex stays within axis_motion per axis of its detected swept segment.
    bool reuse_candidates(Float axis_motion) const;

    Float filter_toi(Float alpha);       // only called by SimEngine
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
