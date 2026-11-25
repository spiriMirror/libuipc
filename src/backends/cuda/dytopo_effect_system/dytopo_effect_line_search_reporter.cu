/*
 * NVIDIA contributions: DyTopo energy producers on side streams and the
 * prelaunch of trial energies.
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

#include <dytopo_effect_system/dytopo_effect_line_search_reporter.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <cuda_tool/cub.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DyTopoEffectLineSearchReporter);

void DyTopoEffectLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.global_dytopo_effect_manager = require<GlobalDyTopoEffectManager>();
}

void DyTopoEffectLineSearchReporter::do_init(LineSearchReporter::InitInfo& info)
{
    m_impl.init();
}

void DyTopoEffectLineSearchReporter::Impl::init() {}

void DyTopoEffectLineSearchReporter::Impl::compute_energy(bool is_init,
                                                          cuda_tool::SideStreams* side_streams,
                                                          bool prelaunched_energy)
{
    auto& manager   = global_dytopo_effect_manager->m_impl;
    auto  reporters = manager.dytopo_effect_reporters.view();

    auto energy_counts = manager.reporter_energy_offsets_counts.counts();
    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalDyTopoEffectManager::EnergyExtentInfo extent_info;
        reporter->report_energy_extent(extent_info);
        energy_counts[i] = extent_info.m_energy_count;
    }

    manager.reporter_energy_offsets_counts.scan();
    SizeT total = manager.reporter_energy_offsets_counts.total_count();

    // The prelaunched energies stand when the counts confirm their ranges
    // and the buffer keeps them; else every energy is launched again (the
    // caller ordered this evaluation after the prelaunch).
    bool keep = prelaunched_energy && total <= energies.capacity();
    for(SizeT i = 0; keep && i < prelaunched.size(); ++i)
    {
        auto [offset, count] = manager.reporter_energy_offsets_counts[i];
        const auto& p        = prelaunched[i];
        SizeT       n        = static_cast<SizeT>(count);
        keep                 = static_cast<SizeT>(offset) == p.offset
               && (p.pending ? n <= p.count : n == p.count);
    }

    loose_resize(energies, total);
    last_total = total;

    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalDyTopoEffectManager::EnergyInfo this_info{side_streams};
        auto [offset, count]    = manager.reporter_energy_offsets_counts[i];
        this_info.m_energies    = energies.view(offset, count);
        this_info.m_is_initial  = is_init;
        this_info.m_prelaunched = keep && i < prelaunched.size();
        reporter->compute_energy(this_info);
    }
    prelaunched.clear();
}

bool DyTopoEffectLineSearchReporter::Impl::prelaunch_energy(cuda_tool::SideStreams* side_streams,
                                                            cudaEvent_t filtered)
{
    auto& manager   = global_dytopo_effect_manager->m_impl;
    auto  reporters = manager.dytopo_effect_reporters.view();

    // The leading reporters that report a prelaunch extent, up to the first
    // whose count is pending on the device: the offsets after that one are
    // known only after the count read-back, so are the energies after it
    // launched (compute_energy).
    prelaunched.clear();
    SizeT end = 0;
    for(auto* reporter : reporters)
    {
        GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo extent_info;
        if(!reporter->report_prelaunch_energy_extent(extent_info))
            break;
        prelaunched.push_back({end, extent_info.m_count, extent_info.m_pending});
        end += extent_info.m_count;
        if(extent_info.m_pending)
            break;
    }
    if(prelaunched.empty())
        return false;

    // The buffer must keep its memory until the evaluation: room for the
    // energies after the prelaunched ones, too (bounded by the last total).
    loose_resize(energies, end + last_total);

    // The pending count's energy (the one that waited for the read-back)
    // leads on the legacy stream, right behind the filters; the others go to
    // side streams that wait for the filters.
    auto launch = [&](SizeT i, bool legacy)
    {
        GlobalDyTopoEffectManager::EnergyInfo this_info{legacy ? nullptr : side_streams,
                                                        legacy ? nullptr : filtered};
        this_info.m_energies =
            energies.view(prelaunched[i].offset, prelaunched[i].count);
        this_info.m_prelaunch = true;
        reporters[i]->compute_energy(this_info);
    };
    SizeT led =
        prelaunched.back().pending ? prelaunched.size() - 1 : prelaunched.size();
    if(led < prelaunched.size())
        launch(led, true);
    for(SizeT i = 0; i < led; ++i)
        launch(i, false);
    return true;
}

void DyTopoEffectLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the record start point for all the vertices we need
}

void DyTopoEffectLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the step forward for all the vertices we need
}

void DyTopoEffectLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace cuda_tool;

    // The reporters write their energies on the side streams.
    m_impl.compute_energy(info.is_initial(), info.side_streams(), info.prelaunched());
}

bool DyTopoEffectLineSearchReporter::do_prelaunch_energy(LineSearcher::ComputeEnergyInfo& info)
{
    return m_impl.prelaunch_energy(info.side_streams(), info.filtered());
}

void DyTopoEffectLineSearchReporter::do_reduce_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace cuda_tool;

    DeviceReduce(info.stream())
        .Sum(m_impl.energies.data(), info.energy().data(), m_impl.energies.size());
}
}  // namespace uipc::backend::cuda
