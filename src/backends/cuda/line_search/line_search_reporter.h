/*
 * NVIDIA contributions: side-stream, early-launch, cached-graph and
 * prelaunched energy interfaces.
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
#include <line_search/line_searcher.h>

namespace uipc::backend::cuda
{
class LineSearchReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

  protected:
    virtual void do_init(InitInfo& info)                                  = 0;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info)    = 0;
    virtual void do_step_forward(LineSearcher::StepInfo& info)            = 0;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) = 0;
    // Optional second phase, called after the producers of the reporter
    // (launched in do_compute_energy on info.side_streams()) were joined into
    // info.stream(): reduce their output into info.energy() with every launch
    // on that stream.
    virtual void do_reduce_energy(LineSearcher::ComputeEnergyInfo& info);
    // Whether the energy reads the trial's active contact set (kept by
    // filter_active). The energy of a reporter that does not (producers and
    // reduction) is launched before that filter (LineSearcher::launch_energy())
    // and, being fixed within a frame (LineSearcher::FixedKey), replayed
    // from a captured graph: such a reporter must launch everything on the
    // streams it is handed and must not read anything back to the host.
    virtual bool do_reads_active_set() const noexcept;
    // Optional, for a reporter that reads the active set: before the
    // read-back of a trial's active-set counts (LineSearcher's prelaunch),
    // launch what of the trial's energy can be launched then, each launch on
    // the legacy stream or on a side stream that waits for
    // info.filtered(), and return true; do_compute_energy then follows with
    // info.prelaunched(), without side streams.
    virtual bool do_prelaunch_energy(LineSearcher::ComputeEnergyInfo& info);


    virtual void do_build(BuildInfo& info);

  private:
    friend class LineSearcher;
    virtual void do_build() override final;
    void         record_start_point(LineSearcher::RecordInfo& info);
    void         step_forward(LineSearcher::StepInfo& info);
    void         compute_energy(LineSearcher::ComputeEnergyInfo& info);
    bool         prelaunch_energy(LineSearcher::ComputeEnergyInfo& info);
    void         reduce_energy(LineSearcher::ComputeEnergyInfo& info);
    bool         reads_active_set() const noexcept;
    void         init();

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
