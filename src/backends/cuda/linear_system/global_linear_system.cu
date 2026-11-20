/*
 * NVIDIA contributions: assembly side streams and prelaunched producers, the
 * FEM gather beside the legacy stream and its in-place writes; the in-place
 * lead's elastic producers prelaunched right after the contact assembly launch
 * and held to the SMs the contact blocks leave, and late Hessian blocks
 * counted after the rest of the assembly is issued and placed behind it.
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

#include <algorithm>
#include <linear_system/global_linear_system.h>
#include <linear_system/diag_linear_subsystem.h>
#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/range.h>
#include <linear_system/iterative_solver.h>
#include <linear_system/global_preconditioner.h>
#include <linear_system/local_preconditioner.h>
#include <fstream>
#include <sim_engine.h>
#include <backends/common/backend_path_tool.h>
#include <Eigen/Sparse>
#include <utils/matrix_market.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalLinearSystem);

GlobalLinearSystem::~GlobalLinearSystem()
{
    if(m_impl.in_place_event)
        cudaEventDestroy(m_impl.in_place_event);
}

SizeT GlobalLinearSystem::dof_count() const
{
    UIPC_ASSERT(m_impl.initialized,
                "GlobalLinearSystem::dof_count() is called before GlobalLinearSystem::init().");
    return m_impl.diag_dof_offsets_counts.total_count();
}

void GlobalLinearSystem::compute_gradient(ComputeGradientInfo& info)
{
    m_impl.compute_gradient(info);
}

void GlobalLinearSystem::do_build()
{
    auto dump_linear_system_attr =
        world().scene().config().find<IndexT>("extras/debug/dump_linear_system");

    m_impl.need_debug_dump =
        dump_linear_system_attr ? dump_linear_system_attr->view()[0] : false;
}

void GlobalLinearSystem::_dump_A_b()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    auto output_path_A = fmt::format("{}A.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_matrix_market(output_path_A, m_impl.bcoo_A.cview());
    logger::info("Dumped global linear system matrix A to {}", output_path_A);

    auto output_path_b = fmt::format("{}b.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_vector_market(output_path_b, m_impl.b.cview());
    logger::info("Dumped global linear system vector b to {}", output_path_b);
}

void GlobalLinearSystem::_dump_x()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    export_vector_market(fmt::format("{}x.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter()),
                         m_impl.x.cview());
}


void GlobalLinearSystem::prepare_prelaunch(SizeT beside_blocks)
{
    m_impl.prepare_prelaunch(beside_blocks);
}

void GlobalLinearSystem::prelaunch_ahead()
{
    m_impl.prelaunch_ahead();
}

void GlobalLinearSystem::prelaunch_assembly(bool contact_launched)
{
    m_impl.prelaunch_assembly(contact_launched);
}

void GlobalLinearSystem::solve()
{
    m_impl.last_solve_iterations = 0;
    m_impl.build_linear_system();

    if(m_impl.empty_system) [[unlikely]]
        return;

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 m_impl.b.size(),
                 m_impl.bcoo_A.triplet_count());

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_A_b();

    m_impl.solve_linear_system();

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_x();

    m_impl.distribute_solution();
}

Float GlobalLinearSystem::diag_norm()
{
    m_impl.build_linear_system();
    return m_impl.diag_norm();
}

Float GlobalLinearSystem::mass_norm()
{
    m_impl.build_linear_system();
    return m_impl.mass_norm();
}

void GlobalLinearSystem::Impl::init()
{
    // 1) Init all diag subsystems and off-diag subsystems

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    {
        // Sort the diag subsystems by their UIDs to ensure the order is consistent
        // ref: https://github.com/spiriMirror/libuipc/issues/271
        std::ranges::sort(diag_subsystem_view,
                          [](const DiagLinearSubsystem* a, const DiagLinearSubsystem* b)
                          { return a->uid() < b->uid(); });
        std::ranges::sort(off_diag_subsystem_view,
                          [](const OffDiagLinearSubsystem* a,
                             const OffDiagLinearSubsystem* b) -> bool
                          { return a->uid() < b->uid(); });
    }


    auto total_count = diag_subsystem_view.size() + off_diag_subsystem_view.size();
    subsystem_infos.resize(total_count);

    // Diag System Always Go First
    auto diag_span = span{subsystem_infos}.subspan(0, diag_subsystem_view.size());
    // Off Diag System Always After Diag System
    auto off_diag_span = span{subsystem_infos}.subspan(diag_subsystem_view.size(),
                                                       off_diag_subsystem_view.size());
    {
        auto offset = 0;
        for(auto i : range(diag_span.size()))
        {
            auto& dst_diag                  = diag_span[i];
            dst_diag.is_diag                = true;
            dst_diag.local_index            = i;
            auto index                      = offset + i;
            dst_diag.index                  = index;
            diag_subsystem_view[i]->m_index = index;
        }

        offset += diag_subsystem_view.size();
        for(auto i : range(off_diag_span.size()))
        {
            auto& dst_off_diag       = off_diag_span[i];
            dst_off_diag.is_diag     = false;
            dst_off_diag.local_index = i;
            dst_off_diag.index       = offset + i;
        }

        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
            diag_subsystem->init();

        for(auto&& [i, off_diag_subsystem] : enumerate(off_diag_subsystem_view))
            off_diag_subsystem->init();
    }


    // 2) DoF Offsets/Counts
    {
        diag_dof_offsets_counts.resize(diag_subsystem_view.size());
        auto diag_dof_counts = diag_dof_offsets_counts.counts();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofExtentInfo info;
            diag_subsystem->report_init_extent(info);
            diag_dof_counts[i] = info.m_dof_count;
            if(info.m_prelaunch_in_place && in_place_lead < 0)
                in_place_lead = static_cast<IndexT>(i);
        }
        diag_dof_offsets_counts.scan();
        auto diag_dof_offsets = diag_dof_offsets_counts.offsets();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofInfo info;
            info.m_dof_offset = diag_dof_offsets[i];
            info.m_dof_count  = diag_dof_counts[i];
            diag_subsystem->receive_init_dof_info(info);
        }
    }
    accuracy_statisfied_flags.resize(diag_subsystem_view.size());

    // 3) Triplet Offsets/Counts
    subsystem_triplet_offsets_counts.resize(total_count);
    off_diag_lr_triplet_counts.resize(off_diag_subsystem_view.size());
    // The in-place lead's range first (its prefix position then does not
    // depend on the other subsystems' counts), the others in their order,
    // then the late blocks (DiagExtentInfo::late_extent()) in subsystem
    // order. No (row, col) key is shared across subsystems, and a
    // subsystem's late blocks stay behind its others: the stable sort of the
    // conversion puts the same entries in the same order under every key,
    // and the reduction sees the same sequence (in its own atomic order).
    triplet_offsets.resize(total_count, 0);
    triplet_order.clear();
    if(in_place_lead >= 0)
        triplet_order.push_back(diag_span[in_place_lead].index);
    for(auto&& [i, subsystem_info] : enumerate(subsystem_infos))
        if(in_place_lead < 0 || subsystem_info.index != diag_span[in_place_lead].index)
            triplet_order.push_back(subsystem_info.index);
    if(in_place_lead >= 0)
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&in_place_event, cudaEventDisableTiming));

    // 4) Preconditioner
    // find out diag systems that don't have preconditioner
    auto local_preconditioner_view = local_preconditioners.view();

    for(auto precond : local_preconditioner_view)
    {
        auto index = precond->m_subsystem->m_index;
        diag_span[index].has_local_preconditioner = true;
    }

    no_precond_diag_subsystem_indices.reserve(diag_span.size());
    for(auto&& [i, diag_info] : enumerate(diag_span))
    {
        if(!diag_info.has_local_preconditioner)
        {
            no_precond_diag_subsystem_indices.push_back(i);
        }
    }

    for(auto precond : local_preconditioner_view)
    {
        precond->init();
    }

    // Enough streams that the scene's longest single-block producer chains
    // (joints, constraints) each get their own; kept below the default of 8
    // hardware work queues together with the legacy stream.
    constexpr SizeT assembly_side_stream_count = 6;
    assembly_streams.create(assembly_side_stream_count);

    initialized = true;
}

void GlobalLinearSystem::Impl::prepare_prelaunch(SizeT beside_blocks)
{
    if(!initialized || assembly_streams.size() == 0)
        return;

    for(auto&& diag_subsystem : diag_subsystems.view())
    {
        PrelaunchInfo info;
        info.m_side_streams = &assembly_streams;
        diag_subsystem->prepare_prelaunch(info);
    }
    prelaunch_prepared = true;
    lead_ahead         = false;
    ahead_blocks       = beside_blocks;
}

void GlobalLinearSystem::Impl::prelaunch_ahead()
{
    // The latency-bound contact kernels take one SM per one-warp block, and
    // an elastic producer of whole-SM blocks issued right after them, held
    // to the SMs they leave, runs beside them (issued after the other DyTopo
    // kernels, it would find no free SM). The in-place lead decides whether
    // it prelaunches here.
    const SizeT beside_blocks = std::exchange(ahead_blocks, 0);
    if(!prelaunch_prepared || beside_blocks == 0 || in_place_lead < 0)
        return;
    if(sm_count == 0)
    {
        int device = 0;
        CUDA_TOOL_CHECK(cudaGetDevice(&device));
        CUDA_TOOL_CHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, device));
    }
    const int beside = static_cast<int>(std::min<SizeT>(beside_blocks, sm_count));

    auto diag_subsystem_view = diag_subsystems.view();
    diag_fully_written_triplets.resize(diag_subsystem_view.size(), 0);
    in_place_written = false;

    PrelaunchInfo info;
    info.m_side_streams      = &assembly_streams;
    info.m_ahead_of_contact  = true;
    info.m_elastic_block_cap = std::max(sm_count / 2, sm_count - beside);
    info.m_sm_count          = sm_count;
    info.m_impl              = this;
    info.m_in_place_lead     = true;
    diag_subsystem_view[in_place_lead]->prelaunch_assembly(info);
    if(!in_place_written)
        return;
    diag_fully_written_triplets[in_place_lead] = info.m_fully_written_triplets;
    lead_ahead                                 = true;
}

void GlobalLinearSystem::Impl::prelaunch_assembly(bool contact_launched)
{
    if(!initialized || assembly_streams.size() == 0)
        return;

    if(!prelaunch_prepared)
        prepare_prelaunch(0);
    prelaunch_prepared = false;

    auto diag_subsystem_view = diag_subsystems.view();
    diag_fully_written_triplets.resize(diag_subsystem_view.size(), 0);
    if(!lead_ahead)
        in_place_written = false;

    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        PrelaunchInfo info;
        info.m_side_streams     = &assembly_streams;
        info.m_contact_launched = contact_launched;
        info.m_impl             = this;
        info.m_in_place_lead    = static_cast<IndexT>(i) == in_place_lead;
        diag_subsystem->prelaunch_assembly(info);
        if(info.m_fully_written_triplets > 0)
            diag_fully_written_triplets[i] = info.m_fully_written_triplets;
    }
    lead_ahead           = false;
    assembly_prelaunched = true;
}

void GlobalLinearSystem::Impl::build_linear_system()
{
    Timer timer{"Build Linear System"};
    empty_system = !_update_subsystem_extent();

    if(empty_system) [[unlikely]]
    {
        // nothing consumes prelaunched producers; finish them before their
        // buffers are reused
        wait_in_place();
        in_place_written = false;
        assembly_streams.join();
        assembly_prelaunched = false;
        std::ranges::fill(diag_fully_written_triplets, 0);
        logger::warn("The global linear system is empty, skip *assembling, *solving and *solution distributing phase.");
        return;
    }

    {
        Timer t{"Assemble Subsystems"};
        _assemble_linear_system();
        assembly_prelaunched = false;
        std::ranges::fill(diag_fully_written_triplets, 0);
        // the next prelaunch may write in place with this layout
        in_place_ready = in_place_lead >= 0;
    }

    {
        Timer t{"Convert To BCOO"};
        converter.ge2sym(triplet_A);
        converter.convert(triplet_A, bcoo_A);
        // upload the nnz count for graph-stable SpMV launches (async on the
        // default stream; drained before any solve reads it)
        triplet_count_dev = (IndexT)bcoo_A.triplet_count();
    }


    {
        Timer t{"Assemble Preconditioner"};
        _assemble_preconditioner();
    }

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 b.size(),
                 bcoo_A.triplet_count());
}

bool GlobalLinearSystem::Impl::_update_subsystem_extent()
{
    bool dof_count_changed     = false;
    bool triplet_count_changed = false;

    auto diag_subsystem_view      = diag_subsystems.view();
    auto off_diag_subsystem_view  = off_diag_subsystems.view();
    auto diag_dof_counts          = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets         = diag_dof_offsets_counts.offsets();
    auto subsystem_triplet_counts = subsystem_triplet_offsets_counts.counts();

    diag_late.assign(diag_subsystem_view.size(), 0);
    diag_late_counts.assign(diag_subsystem_view.size(), 0);
    late_pending  = false;
    late_reported = false;

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto           dof_i          = subsystem_info.local_index;
            auto           triplet_i      = subsystem_info.index;
            auto&          diag_subsystem = diag_subsystem_view[dof_i];
            DiagExtentInfo info;
            diag_subsystem->report_extent(info);

            dof_count_changed |= diag_dof_counts[dof_i] != info.m_dof_count;
            diag_dof_counts[dof_i] = info.m_dof_count;
            diag_late[dof_i]       = info.m_late;
            late_pending |= info.m_late;


            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != info.m_block_count;
            subsystem_triplet_counts[triplet_i] = info.m_block_count;
        }
        else
        {
            auto triplet_i = subsystem_info.index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
            OffDiagExtentInfo info;
            off_diag_subsystem->report_extent(info);

            auto total_block_count = info.m_lr_block_count + info.m_rl_block_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != total_block_count;
            subsystem_triplet_counts[triplet_i] = total_block_count;
            off_diag_lr_triplet_counts[subsystem_info.local_index] =
                SizeT2{info.m_lr_block_count, info.m_rl_block_count};
        }
    }

    SizeT total_dof     = 0;
    SizeT total_triplet = 0;

    // An in-place prelaunch used the previous DoF layout: when it changed,
    // the lead's writes are void (and b may move), so wait for them and let
    // the lead assemble in full.
    in_place_discarded = false;
    if(in_place_written && dof_count_changed)
    {
        wait_in_place();
        in_place_written                           = false;
        in_place_discarded                         = true;
        diag_fully_written_triplets[in_place_lead] = 0;
    }

    if(dof_count_changed)
    {
        diag_dof_offsets_counts.scan();
    }
    total_dof = diag_dof_offsets_counts.total_count();
    if(x.capacity() < total_dof)
    {
        auto reserve_count = total_dof * reserve_ratio;
        x.reserve(reserve_count);
        b.reserve(reserve_count);
    }
    auto blocked_dof = total_dof / DoFBlockSize;
    triplet_A.reshape(blocked_dof, blocked_dof);
    x.resize(total_dof);
    b.resize(total_dof);

    if(triplet_count_changed) [[likely]]
    {
        subsystem_triplet_offsets_counts.scan();
        IndexT offset = 0;
        for(SizeT t : triplet_order)
        {
            triplet_offsets[t] = offset;
            offset += subsystem_triplet_counts[t];
        }
    }
    total_triplet = subsystem_triplet_offsets_counts.total_count();

    if(triplet_A.triplet_capacity() < total_triplet)
    {
        auto reserve_count = total_triplet * reserve_ratio;
        if(in_place_written)
        {
            // keep the in-place prefix (the first triplets): copy it once
            // the lead has written it
            wait_in_place();
            SizeT prefix = diag_fully_written_triplets[in_place_lead];
            triplet_A.resize_triplets_discard(std::max(triplet_A.triplet_count(), prefix));
            triplet_A.reserve_triplets(reserve_count);
        }
        else
            triplet_A.reserve_triplets_discard(reserve_count);
        bcoo_A.reserve_triplets_discard(reserve_count);
    }
    triplet_A.resize_triplets_discard(total_triplet);

    // without other blocks, only the late ones tell whether A is empty
    if(total_triplet == 0 && late_pending && total_dof != 0) [[unlikely]]
        total_triplet = _report_late_extents();

    if(total_dof == 0 || total_triplet == 0) [[unlikely]]
    {
        return false;
    }

    return true;
}

SizeT GlobalLinearSystem::Impl::_report_late_extents()
{
    SizeT total = 0;
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        if(!diag_late[i])
            continue;
        LateExtentInfo info;
        diag_subsystem->report_late_extent(info);
        diag_late_counts[i] = info.m_block_count;
        total += info.m_block_count;
    }
    late_reported = true;
    return total;
}

void GlobalLinearSystem::Impl::_assemble_late()
{
    if(!late_pending)
        return;
    if(!late_reported)
        _report_late_extents();

    const SizeT early = subsystem_triplet_offsets_counts.total_count();
    SizeT       total = early;
    for(SizeT count : diag_late_counts)
        total += count;
    if(triplet_A.triplet_capacity() < total)
    {
        // keep the other blocks: copy them once they are written
        wait_in_place();
        assembly_streams.join();
        auto reserve_count = total * reserve_ratio;
        triplet_A.reserve_triplets(reserve_count);
        bcoo_A.reserve_triplets_discard(reserve_count);
        assembly_streams.fork();
    }
    triplet_A.resize_triplets_discard(total);

    auto   HA               = triplet_A.view();
    auto   B                = b.view();
    auto   diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto   diag_dof_offsets = diag_dof_offsets_counts.offsets();
    IndexT offset           = static_cast<IndexT>(early);
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        if(!diag_late[i])
            continue;
        int  dof_offset         = diag_dof_offsets[i];
        int  dof_count          = diag_dof_counts[i];
        int  blocked_dof_offset = dof_offset / DoFBlockSize;
        int  blocked_dof_count  = dof_count / DoFBlockSize;
        int2 ij_offset          = {blocked_dof_offset, blocked_dof_offset};
        int2 ij_count           = {blocked_dof_count, blocked_dof_count};
        auto count              = static_cast<IndexT>(diag_late_counts[i]);

        DiagInfo info{this};
        info.m_index        = diag_subsystem->m_index;
        info.m_side_streams = &assembly_streams;
        info.m_prelaunched =
            assembly_prelaunched
            && !(in_place_discarded && static_cast<IndexT>(i) == in_place_lead);
        info.m_gradients = B.subview(dof_offset, dof_count);
        info.m_hessians = HA.subview(offset, count).submatrix(ij_offset, ij_count);
        diag_subsystem->assemble_late(info);
        offset += count;
    }
}

void GlobalLinearSystem::Impl::_clear_triplets()
{
    auto HA    = triplet_A.view();
    auto clear = [&](IndexT begin, IndexT end)
    {
        if(end <= begin)
            return;
        auto range = HA.subview(begin, end - begin);
        range.values().fill(Matrix3x3::Zero());
        range.row_indices().fill(-1);
        range.col_indices().fill(-1);
    };

    // Clear and invalidate previous values, except the prefixes that
    // prelaunched subsystems write in full (in triplet order).
    auto   offsets = span<const IndexT>{triplet_offsets};
    auto   counts  = subsystem_triplet_offsets_counts.counts();
    IndexT begin   = 0;
    if(assembly_prelaunched)
    {
        for(SizeT t : triplet_order)
        {
            const auto& subsystem_info = subsystem_infos[t];
            if(!subsystem_info.is_diag)
                continue;
            SizeT written = diag_fully_written_triplets[subsystem_info.local_index];
            if(written == 0)
                continue;
            IndexT offset = offsets[subsystem_info.index];
            UIPC_ASSERT(written <= static_cast<SizeT>(counts[subsystem_info.index]),
                        "Fully written prefix {} exceeds the subsystem's {} triplets",
                        written,
                        counts[subsystem_info.index]);
            clear(begin, offset);
            begin = offset + static_cast<IndexT>(written);
        }
    }
    clear(begin, static_cast<IndexT>(triplet_A.triplet_count()));
}

void GlobalLinearSystem::Impl::_assemble_linear_system()
{
    auto HA = triplet_A.view();

    _clear_triplets();

    // An in-place lead cleared (and is adding to) its gradient range.
    auto B = b.view();
    if(in_place_written)
    {
        IndexT off = diag_dof_offsets_counts.offsets()[in_place_lead];
        IndexT cnt = diag_dof_offsets_counts.counts()[in_place_lead];
        if(off > 0)
            B.buffer_view().subview(0, off).fill(0.0);
        if(off + cnt < (IndexT)B.size())
            B.buffer_view().subview(off + cnt, B.size() - off - cnt).fill(0.0);
    }
    else
        B.buffer_view().fill(0.0);

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    auto subsystem_triplet_counts  = subsystem_triplet_offsets_counts.counts();
    auto subsystem_triplet_offsets = span<const IndexT>{triplet_offsets};

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto  dof_i          = subsystem_info.local_index;
            auto  triplet_i      = subsystem_info.index;
            auto& diag_subsystem = diag_subsystem_view[dof_i];

            int  dof_offset         = diag_dof_offsets[dof_i];
            int  dof_count          = diag_dof_counts[dof_i];
            int  blocked_dof_offset = dof_offset / DoFBlockSize;
            int  blocked_dof_count  = dof_count / DoFBlockSize;
            int2 ij_offset          = {blocked_dof_offset, blocked_dof_offset};
            int2 ij_count           = {blocked_dof_count, blocked_dof_count};

            DiagInfo info{this};

            info.m_index        = triplet_i;
            info.m_side_streams = &assembly_streams;
            info.m_prelaunched =
                assembly_prelaunched
                && !(in_place_discarded && static_cast<IndexT>(dof_i) == in_place_lead);
            info.m_gradients = B.subview(dof_offset, dof_count);
            info.m_hessians  = HA.subview(subsystem_triplet_offsets[triplet_i],
                                         subsystem_triplet_counts[triplet_i])
                                  .submatrix(ij_offset, ij_count);

            diag_subsystem->assemble(info);
        }
        else
        {
            auto triplet_i   = subsystem_info.index;
            auto local_index = subsystem_info.local_index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
            auto& l_diag_index = off_diag_subsystem->m_l->m_index;
            auto& r_diag_index = off_diag_subsystem->m_r->m_index;


            int l_blocked_dof_offset = diag_dof_offsets[l_diag_index] / DoFBlockSize;
            int l_blocked_dof_count = diag_dof_counts[l_diag_index] / DoFBlockSize;

            int r_blocked_dof_offset = diag_dof_offsets[r_diag_index] / DoFBlockSize;
            int r_blocked_dof_count = diag_dof_counts[r_diag_index] / DoFBlockSize;

            auto lr_triplet_offset = subsystem_triplet_offsets[triplet_i];
            auto lr_triplet_count  = off_diag_lr_triplet_counts[local_index].x;
            auto rl_triplet_offset = lr_triplet_offset + lr_triplet_count;
            auto rl_triplet_count  = off_diag_lr_triplet_counts[local_index].y;

            OffDiagInfo info{this};
            info.m_index = triplet_i;

            info.m_lr_hessian =
                HA.subview(lr_triplet_offset, lr_triplet_count)
                    .submatrix(int2{l_blocked_dof_offset, r_blocked_dof_offset},
                               int2{l_blocked_dof_count, r_blocked_dof_count});

            info.m_rl_hessian =
                HA.subview(rl_triplet_offset, rl_triplet_count)
                    .submatrix(int2{r_blocked_dof_offset, l_blocked_dof_offset},
                               int2{r_blocked_dof_count, l_blocked_dof_count});

            // logger::info("rl_offset: {}, lr_offset: {}", rl_triplet_offset, lr_triplet_offset);

            off_diag_subsystem->assemble(info);
        }
    }

    // the late blocks, behind the others
    _assemble_late();

    // Subsystems write disjoint ranges of A and b; producers they forked onto
    // side streams (and an in-place prelaunch) are complete before the
    // conversion reads A.
    wait_in_place();
    in_place_written   = false;
    in_place_discarded = false;
    assembly_streams.join();
}


void GlobalLinearSystem::Impl::wait_in_place()
{
    if(!in_place_written || in_place_waited)
        return;
    CUDA_TOOL_CHECK(cudaStreamWaitEvent(nullptr, in_place_event, 0));
    in_place_waited = true;
}

void GlobalLinearSystem::Impl::_assemble_preconditioner()
{
    if(global_preconditioner)
    {
        GlobalPreconditionerAssemblyInfo info{this};
        global_preconditioner->assemble(info);
    }

    // Local preconditioners only read the assembled system and write their
    // own state, so each one assembles on its own side stream; the solve
    // starts after the join.
    assembly_streams.fork();
    for(auto&& [i, preconditioner] : enumerate(local_preconditioners.view()))
    {
        cudaStream_t stream = assembly_streams.size() ?
                                  assembly_streams[i % assembly_streams.size()] :
                                  nullptr;
        LocalPreconditionerAssemblyInfo info{this, preconditioner->m_subsystem->m_index, stream};
        preconditioner->assemble(info);
    }
    assembly_streams.join();
}

void GlobalLinearSystem::Impl::solve_linear_system()
{
    Timer timer{"Solve Linear System"};
    if(iterative_solver)
    {
        SolvingInfo info{this};
        info.m_b = b.cview();
        info.m_x = x.view();
        iterative_solver->solve(info);
        last_solve_iterations = info.m_iter_count;
        logger::info("Iterative linear solver iteration count: {}", info.m_iter_count);
    }
}

void GlobalLinearSystem::Impl::distribute_solution()
{
    auto diag_subsystem_view = diag_subsystems.view();
    auto diag_dof_counts     = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets    = diag_dof_offsets_counts.offsets();

    // distribute the solution to all diag subsystems
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        SolutionInfo info{this};
        info.m_solution = x.view().subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystem->retrieve_solution(info);
    }
}

void GlobalLinearSystem::Impl::apply_preconditioner(cuda_tool::DenseVectorView<Float> z,
                                                    cuda_tool::CDenseVectorView<Float> r,
                                                    cuda_tool::CVarView<IndexT> converged,
                                                    cudaStream_t stream)
{
    (void)converged;
    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    if(global_preconditioner)
    {
        ApplyPreconditionerInfo info{this};
        info.m_z         = z;
        info.m_r         = r;
        info.m_converged = converged;
        info.m_stream    = stream;
        global_preconditioner->apply(info);
    }

    for(auto& preconditioner : local_preconditioners.view())
    {
        ApplyPreconditionerInfo info{this};
        auto                    index  = preconditioner->m_subsystem->m_index;
        auto                    offset = diag_dof_offsets[index];
        auto                    count  = diag_dof_counts[index];
        info.m_z                       = z.subview(offset, count);
        info.m_r                       = r.subview(offset, count);
        info.m_converged               = converged;
        info.m_stream                  = stream;
        preconditioner->apply(info);
    }

    if(!global_preconditioner)
    {
        // For diag subsystems without local preconditioner, just copy r to z
        for(auto i : no_precond_diag_subsystem_indices)
        {
            auto offset = diag_dof_offsets[i];
            auto count  = diag_dof_counts[i];
            auto z_sub  = z.subview(offset, count);
            auto r_sub  = r.subview(offset, count);
            cuda_tool::BufferLaunch(stream).copy(z_sub.buffer_view(), r_sub.buffer_view());
        }
    }
}

void GlobalLinearSystem::Impl::spmv(Float                              a,
                                    cuda_tool::CDenseVectorView<Float> x,
                                    Float                              b,
                                    cuda_tool::DenseVectorView<Float>  y)
{
    spmver.rbk_sym_spmv(a, bcoo_A.cview(), x, b, y);

    // Just some debug options
    //  * spmver.sym_spmv(a, bcoo_A.cview(), x, b, y);      // Slightly slower
    //  * spmver.cpu_sym_spmv(a, bcoo_A.cview(), x, b, y);  // Much slower
}

void GlobalLinearSystem::Impl::spmv_dot(cuda_tool::CDenseVectorView<Float> x,
                                        cuda_tool::DenseVectorView<Float>  y,
                                        cuda_tool::VarView<Float> d_dot,
                                        cudaStream_t              stream)
{
    spmver.rbk_sym_spmv_dot(1.0,
                            bcoo_A.cview(),
                            x,
                            0.0,
                            y,
                            d_dot,
                            triplet_count_dev.cviewer(),
                            bcoo_A.triplet_capacity(),
                            stream);
}

bool GlobalLinearSystem::Impl::accuracy_statisfied(cuda_tool::DenseVectorView<Float> r)
{
    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    for(auto&& [i, diag_subsystems] : enumerate(diag_subsystems.view()))
    {
        AccuracyInfo info{this};
        info.m_r = r.subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystems->accuracy_check(info);

        accuracy_statisfied_flags[i] = info.m_statisfied ? 1 : 0;
    }

    return std::ranges::all_of(accuracy_statisfied_flags,
                               [](bool flag) { return flag; });
}

void GlobalLinearSystem::Impl::compute_gradient(ComputeGradientInfo& info)
{
    auto diag_subsystem_view = diag_subsystems.view();

    // report extent first (gradient only mode)
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagExtentInfo diag_info;
        diag_info.m_gradient_only   = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->report_extent(diag_info);
    }

    // assemble gradient only
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagInfo diag_info{this};
        diag_info.m_index           = diag_subsystem->m_index;
        diag_info.m_gradients       = info.m_gradients;
        diag_info.m_hessians        = TripletMatrixView{};
        diag_info.m_gradient_only   = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->assemble(diag_info);
    }
}

Float GlobalLinearSystem::Impl::diag_norm()
{
    Float norm = 0;

    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        DiagNormInfo info(this, diag_subsystem->m_index);
        norm = max(norm, diag_subsystem->diag_norm(info));
    }

    return norm;
}

Float GlobalLinearSystem::Impl::mass_norm()
{
    Float norm = 0;

    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        DiagNormInfo info(this, diag_subsystem->m_index);
        norm = max(norm, diag_subsystem->mass_norm(info));
    }

    return norm;
}

void GlobalLinearSystem::DiagExtentInfo::extent(SizeT hessian_block_count, SizeT dof_count) noexcept
{
    m_block_count = hessian_block_count;
    UIPC_ASSERT(dof_count % DoFBlockSize == 0,
                "dof_count must be multiple of {}, yours {}.",
                DoFBlockSize,
                dof_count);
    m_dof_count = dof_count;
}

void GlobalLinearSystem::OffDiagExtentInfo::extent(SizeT lr_hessian_block_count,
                                                   SizeT rl_hassian_block_count) noexcept
{
    m_lr_block_count = lr_hessian_block_count;
    m_rl_block_count = rl_hassian_block_count;
}
bool GlobalLinearSystem::PrelaunchInfo::in_place_available() const noexcept
{
    return m_in_place_lead && m_impl && m_impl->in_place_ready;
}

auto GlobalLinearSystem::PrelaunchInfo::in_place_hessians(SizeT count) -> TripletMatrixView
{
    UIPC_ASSERT(in_place_available(), "in-place prelaunch is not available");
    auto& I = *m_impl;
    // Nothing reads the previous assembly's triplets any more (the
    // reallocation synchronizes), and the global extent update keeps the
    // prefix when it grows the matrix after this.
    if(I.triplet_A.triplet_capacity() < count)
        I.triplet_A.reserve_triplets_discard(count * I.reserve_ratio);
    if(I.triplet_A.triplet_count() < count)
        I.triplet_A.resize_triplets_discard(count);
    I.in_place_written = true;
    I.in_place_waited  = false;

    int offset = I.diag_dof_offsets_counts.offsets()[I.in_place_lead] / DoFBlockSize;
    int blocks = I.diag_dof_offsets_counts.counts()[I.in_place_lead] / DoFBlockSize;
    return I.triplet_A.view().subview(0, count).submatrix(int2{offset, offset},
                                                          int2{blocks, blocks});
}

auto GlobalLinearSystem::PrelaunchInfo::in_place_gradients() -> DenseVectorView
{
    UIPC_ASSERT(in_place_available(), "in-place prelaunch is not available");
    auto& I = *m_impl;
    return I.b.view().subview(I.diag_dof_offsets_counts.offsets()[I.in_place_lead],
                              I.diag_dof_offsets_counts.counts()[I.in_place_lead]);
}

int GlobalLinearSystem::PrelaunchInfo::in_place_block_offset() const
{
    return m_impl->diag_dof_offsets_counts.offsets()[m_impl->in_place_lead] / DoFBlockSize;
}

void GlobalLinearSystem::PrelaunchInfo::in_place_done(cudaStream_t stream)
{
    UIPC_ASSERT(m_impl && m_impl->in_place_written, "no in-place prelaunch to finish");
    CUDA_TOOL_CHECK(cudaEventRecord(m_impl->in_place_event, stream));
}

auto GlobalLinearSystem::AssemblyInfo::A() const -> CBCOOMatrixView
{
    return m_impl->bcoo_A.cview();
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_offset() const
{
    auto diag_dof_offsets = m_impl->diag_dof_offsets_counts.offsets();
    return diag_dof_offsets[m_index];
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_count() const
{
    auto diag_dof_counts = m_impl->diag_dof_offsets_counts.counts();
    return diag_dof_counts[m_index];
}

SizeT GlobalLinearSystem::last_solve_iterations() const noexcept
{
    return m_impl.last_solve_iterations;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void GlobalLinearSystem::add_subsystem(DiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    UIPC_ASSERT(subsystem != nullptr, "The subsystem should not be nullptr.");
    m_impl.diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_subsystem(OffDiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    m_impl.off_diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_solver(IterativeSolver* solver)
{
    check_state(SimEngineState::BuildSystems, "add_solver()");
    UIPC_ASSERT(solver != nullptr, "The solver should not be nullptr.");
    m_impl.iterative_solver.register_sim_system(*solver);
}

void GlobalLinearSystem::add_preconditioner(LocalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.local_preconditioners.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::add_preconditioner(GlobalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.global_preconditioner.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::init()
{
    m_impl.init();
}

void GlobalLinearSystem::ComputeGradientInfo::flags(ComponentFlags flags) noexcept
{
    m_flags = flags;
}

void GlobalLinearSystem::ComputeGradientInfo::buffer_view(cuda_tool::DenseVectorView<Float> grad) noexcept
{
    m_gradients = grad;
}
}  // namespace uipc::backend::cuda
