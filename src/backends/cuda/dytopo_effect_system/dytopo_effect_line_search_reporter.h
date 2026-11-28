/*
 * NVIDIA contributions: side-stream and prelaunched energy interfaces.
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
#include <line_search/line_search_reporter.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <dytopo_effect_system/dytopo_effect_reporter.h>

namespace uipc::backend::cuda
{
class DyTopoEffectLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl;

    class Impl
    {
      public:
        void init();
        void compute_energy(bool is_init, cuda_tool::SideStreams* side_streams, bool prelaunched_energy);
        bool prelaunch_energy(cuda_tool::SideStreams* side_streams, cudaEvent_t filtered);

        SimSystemSlot<GlobalDyTopoEffectManager> global_dytopo_effect_manager;

        cuda_tool::DeviceBuffer<Float> energies;
        Float                          reserve_ratio = 1.5;
        SizeT                          last_total    = 0;

        // The leading reporters whose trial energies prelaunch_energy()
        // launched: the range each was launched into (a pending count's
        // range spans its capacity).
        struct Prelaunched
        {
            SizeT offset  = 0;
            SizeT count   = 0;
            bool  pending = false;
        };
        vector<Prelaunched> prelaunched;

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

    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;

  private:
    virtual void do_init(LineSearchReporter::InitInfo& info) override;
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) override;
    virtual bool do_prelaunch_energy(LineSearcher::ComputeEnergyInfo& info) override;
    virtual void do_reduce_energy(LineSearcher::ComputeEnergyInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
