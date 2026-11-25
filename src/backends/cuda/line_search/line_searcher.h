/*
 * NVIDIA contributions: interfaces and state of the concurrent and cached
 * energy evaluation, the prologue stream, batched copies and step forward,
 * and the prelaunched trial energies.
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
#include <cuda_tool/buffer.h>
#include <cuda_tool/copy_batch.h>
#include <cuda_tool/graph.h>
#include <line_search/step_batch.h>
#include <optional>

namespace uipc::backend::cuda
{
class LineSearchReporter;
class GlobalAnimator;

class LineSearcher : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    ~LineSearcher() override;

    class RecordInfo
    {
      public:
        // The start-point copies of all reporters (and the vertex
        // manager's) run as one batch after every reporter recorded.
        cuda_tool::CopyBatch& copies() const noexcept { return *m_copies; }

      private:
        friend class LineSearcher;
        cuda_tool::CopyBatch* m_copies = nullptr;
    };

    class StepInfo
    {
      public:
        Float alpha;
        // The batch the step joins (see LineSearcher::step_forward(StepBatch&));
        // nullptr: the reporter launches its step itself.
        StepBatch* batch() const noexcept { return m_batch; }

      private:
        friend class LineSearcher;
        StepBatch* m_batch = nullptr;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(LineSearcher* impl, cuda_tool::VarView<Float> energy) noexcept;
        Float                     dt() noexcept;
        cuda_tool::VarView<Float> energy() noexcept;
        bool                      is_initial() noexcept;
        // Side streams forked for this evaluation. Each independent energy
        // producer takes one (cuda_tool::LazySideStream) for all of its
        // launches; LineSearcher joins them into stream() before the
        // reporter reduces its output there. Null after a prelaunch: the
        // producers then launch on the legacy stream.
        cuda_tool::SideStreams* side_streams() noexcept;
        // Stream of the reporter's reduction
        // (LineSearchReporter::do_reduce_energy): every launch of the
        // reduction goes here.
        cudaStream_t stream() noexcept;
        // The prelaunch (LineSearchReporter::do_prelaunch_energy): the event
        // after the trial's filter_active work on the legacy stream; the
        // side streams were not forked.
        cudaEvent_t filtered() noexcept;
        // The reporter's prelaunch launched this evaluation's producers.
        bool prelaunched() noexcept;


      private:
        friend class LineSearcher;
        LineSearcher*             m_impl = nullptr;
        cuda_tool::VarView<Float> m_energy;
        bool                      m_energy_set   = false;
        bool                      m_is_initial   = false;
        bool                      m_prelaunched  = false;
        cuda_tool::SideStreams*   m_side_streams = nullptr;
        cudaStream_t              m_stream       = cuda_tool::default_stream();
        cudaEvent_t               m_filtered     = nullptr;
    };

    void add_reporter(LineSearchReporter* reporter);

    SizeT max_iter() const noexcept;
    // line_search/cubic_refinement: refine a backtracked step once at the
    // minimizer of a cubic fit (off by default).
    bool cubic_refinement() const noexcept;

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    friend class ContactExporterManager;
    void refresh_contact_energies();
    void compute_active_set_energies(bool                    is_initial,
                                     bool                    prelaunched,
                                     cuda_tool::SideStreams* side_streams);
    void init();                // only be called by SimEngine
    void record_start_point();  // only be called by SimEngine
    // The reporters' copies join `copies` (launched by the caller).
    void record_start_point(cuda_tool::CopyBatch& copies);  // only be called by SimEngine
    void step_forward(Float alpha);  // only be called by SimEngine
    // The reporters' steps join `steps` (launched by the caller).
    void  step_forward(StepBatch& steps);   // only be called by SimEngine
    Float compute_energy(bool is_initial);  // only be called by SimEngine
    // compute_energy(true) in two halves (only called by SimEngine):
    // launch_initial_energy() enqueues the evaluation and the copy of its
    // energies to the host without waiting for them; initial_energy() waits
    // and returns the total. In between, the next trial may step forward and
    // launch its energy: stream order keeps the copy ahead of every write to
    // the energy slots.
    void  launch_initial_energy();
    Float initial_energy();
    // Pipelined trial (only called by SimEngine): right after a trial stepped
    // forward, launch_energy() launches the energy of the reporters that do
    // not read the active contact set (the fixed part below), so it runs
    // beside the trial's filter_active and, for the first trial, the CCD TOI;
    // the next compute_energy(false) launches the others after that filter
    // and completes the evaluation. discard_energy() drops a launched
    // evaluation whose trial is stepped again or not evaluated.
    void launch_energy();
    void discard_energy();
    // Called by SimEngine through GlobalTrajectoryFilter::filter_active's
    // before_sync when a trial energy follows the filter (it may still be
    // discarded): the reporters that read the active set launch what they
    // can before the read-back of its counts (the contact energies by the
    // capacity of the pending active set), and the next compute_energy(false)
    // completes the evaluation behind them.
    void prelaunch_energy(cudaEvent_t filtered);

    // The fixed part of an evaluation: the producers of the reporters that
    // do not read the active contact set (ABD and FEM) on m_fixed_streams
    // and their reductions on m_fixed_streams[0], into the reporters'
    // energy slots. Their launches depend on the frame only through device
    // memory: their by-value arguments are dt, the animation substep ratio
    // and the extents (element counts, energy ranges) of the producers,
    // whose only per-frame ones are the animator constraints' energy
    // counts, and their buffers keep their device memory while no device
    // allocation changes (fixed_key()). So after one direct evaluation with
    // a key they are captured into m_fixed_graph and later evaluations with
    // that key, in any frame, replay it: one launch instead of the
    // producers', reductions' and combine kernels' launches, with the same
    // kernels, parameters and dependencies. A failed capture (a producer
    // launching on the legacy stream invalidates it) disables the graph:
    // every evaluation then launches directly.
    struct FixedKey
    {
        Float dt            = 0.0;
        Float substep_ratio = 0.0;
        SizeT extent_generation = ~0ull;  // GlobalAnimator::energy_extent_generation()
        std::uint64_t allocation_epoch = ~0ull;  // cuda_tool::device_allocation_epoch()
        bool operator==(const FixedKey&) const = default;
    };
    enum class FixedGraphState
    {
        Stale,  // launch directly, then capture with the same key
        Warm,   // the last direct launch had this key: capture and replay
        Ready,  // replay
    };
    FixedKey fixed_key();
    void     launch_fixed_energy();
    // Everything of an evaluation up to the total on the default stream. The
    // total goes to total_out (device-accessible, e.g. a ReadbackBuffer's
    // device_data()), by default to the device slot after the reporters'.
    void enqueue_energy(bool is_initial, Float* total_out = nullptr);
    Float read_energy(span<const Float> values);  // checks (and reports) the energies
    // The fixed launches, ordered after the work enqueued on `origin` so
    // far. Direct (origin = the legacy stream): records m_fixed_done.
    // Capturing (origin = the capture stream): ends joined into `origin`.
    void record_fixed_energy(cudaStream_t origin, bool capturing);

    SimSystemSlotCollection<LineSearchReporter> m_reporters;

    // Line-search prologue of SimEngine::advance(): the feasible step and
    // the Newton tolerance checks run on this stream beside the candidate
    // reuse check, the CCD TOI beside E_0 and the first trial.
    cuda_tool::SideStream m_prologue_stream;

    // Side streams of the energy producers that read the active contact
    // set, forked once per evaluation.
    cuda_tool::SideStreams m_energy_streams;
    vector<IndexT> m_energy_set;  // per reporter: energy() taken by the producers or the reduction
    bool m_energy_launched = false;  // by launch_energy()
    // by prelaunch_energy(), and per reporter
    bool                 m_energy_prelaunched = false;
    std::optional<SizeT> m_skipped_contact_energy_frame;
    vector<IndexT>       m_reporter_prelaunched;

    // The fixed part (see FixedKey): the producers' side streams and one
    // reduce stream per fixed reporter (a reporter's reductions wait only
    // for the side streams its producers took). These streams carry nothing
    // else, so the CUB workspaces of the reduce streams that the graph
    // keeps are its own.
    cuda_tool::SideStreams  m_fixed_streams;
    cuda_tool::SideStreams  m_fixed_reduce_streams;
    cuda_tool::GraphCapture m_fixed_graph;
    FixedGraphState         m_fixed_state = FixedGraphState::Stale;
    FixedKey                m_fixed_key;
    cudaEvent_t             m_fixed_done = nullptr;  // the fixed part completed
    bool                    m_has_fixed_energy = false;
    SimSystemSlot<GlobalAnimator> m_global_animator;

    cuda_tool::DeviceBuffer<Float>     m_device_energy_values;
    vector<Float>                      m_energy_values;
    cuda_tool::PinnedHostBuffer<Float> m_initial_energy_values;
    // compute_energy()'s total, stored by the final sum
    cuda_tool::ReadbackBuffer<Float>        m_total_energy;
    cudaEvent_t                             m_initial_energy_ready   = nullptr;
    bool                                    m_initial_energy_pending = false;
    bool                                    m_report_energy          = false;
    std::stringstream                       m_report_stream;
    S<const geometry::AttributeSlot<Float>> m_dt_attr;
    IndexT                                  m_max_iter         = 64;
    bool                                    m_cubic_refinement = false;
};
}  // namespace uipc::backend::cuda
