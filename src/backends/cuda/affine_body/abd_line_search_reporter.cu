/*
 * NVIDIA contributions: line-search energy producers on side streams, the
 * cached energy graph, batched start-point copies and the batched step
 * forward.
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

#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <affine_body/abd_line_search_subreporter.h>
#include <affine_body/affine_body_kinetic.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_line_search_reporter_step_forward_kernel(
        cuda_tool::CBufferView<IndexT>   is_fixed,
        cuda_tool::CBufferView<Vector12> q_temps,
        cuda_tool::BufferView<Vector12>  qs,
        cuda_tool::CBufferView<Vector12> dqs,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(is_fixed(i))
            return;
        qs(i) = q_temps(i) + alpha * dqs(i);
    }

    __global__ void abd_line_search_reporter_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT> is_fixed,
        cuda_tool::CBufferView<IndexT> external_kinetic,
        cuda_tool::BufferView<Float>   kinetic_energy,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(is_fixed(i) || external_kinetic(i))
        {
            kinetic_energy(i) = 0.0;
        }
    }

    __global__ void abd_line_search_reporter_combine_energy_kernel(
        cuda_tool::CVarView<Float> kinetic_energy,
        cuda_tool::CVarView<Float> shape_energy,
        cuda_tool::CVarView<Float> reporter_energy,
        cuda_tool::VarView<Float>  total_energy)
    {
        if(blockIdx.x != 0 || threadIdx.x != 0)
            return;
        *total_energy = *kinetic_energy + *shape_energy + *reporter_energy;
    }
}  // namespace

REGISTER_SIM_SYSTEM(ABDLineSearchReporter);

void ABDLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
}

void ABDLineSearchReporter::Impl::init(LineSearchReporter::InitInfo& info)
{
    auto reporter_view = reporters.view();
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;  // Assign index for each reporter
    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();

    reporter_energy_offsets_counts.resize(reporter_view.size());
}

void ABDLineSearchReporter::Impl::record_start_point(LineSearcher::RecordInfo& info)
{
    info.copies().copy<Vector12>(abd().body_id_to_q_temp.view(),
                                 std::as_const(abd().body_id_to_q).view());
}

void ABDLineSearchReporter::Impl::step_forward(LineSearcher::StepInfo& info)
{
    int n = (int)abd().abd_body_count;
    if(auto steps = info.batch())
    {
        steps->dofs(abd().body_id_to_q.view(),
                    std::as_const(abd().body_id_to_q_temp).view(),
                    std::as_const(abd().body_id_to_dq).view(),
                    std::as_const(abd().body_id_to_is_fixed).view(),
                    n);
        return;
    }
    if(n > 0)
    {
        auto k = abd_line_search_reporter_step_forward_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            abd().body_id_to_is_fixed.cview(),
            abd().body_id_to_q_temp.cview(),
            abd().body_id_to_q.view(),
            abd().body_id_to_dq.cview(),
            info.alpha,
            n);
    }
}

void ABDLineSearchReporter::Impl::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace cuda_tool;

    auto body_count   = abd().body_count();
    auto side_streams = info.side_streams();

    // The kinetic, shape and reporter energies are written on the side
    // streams (one per producer); reduce_energy() sums them on the default
    // stream after LineSearcher joined the producers.

    // Compute kinetic energy
    {
        body_id_to_kinetic_energy.resize_discard(body_count);

        ABDLineSearchReporter::ComputeEnergyInfo this_info{side_streams};
        this_info.m_energies = body_id_to_kinetic_energy.view();
        this_info.m_dt       = info.dt();

        abd().kinetic->compute_energy(this_info);
    }

    // Compute shape energy
    {
        body_id_to_shape_energy.resize_discard(body_count);

        // Distribute the computation of shape energy to each constitution
        for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
        {
            auto shape_energy = abd().subview(body_id_to_shape_energy, cst->m_index);

            ABDLineSearchReporter::ComputeEnergyInfo this_info{side_streams};
            this_info.m_energies = shape_energy;
            this_info.m_dt       = info.dt();
            cst->compute_energy(this_info);
        }
    }

    // Collect the energy from other reporters
    {
        auto         reporter_view = reporters.view();
        span<IndexT> counts        = reporter_energy_offsets_counts.counts();
        for(auto&& [i, R] : enumerate(reporter_view))
        {
            ReportExtentInfo this_info;
            R->report_extent(this_info);
            counts[i] = this_info.m_energy_count;
        }

        reporter_energy_offsets_counts.scan();
        reporter_energies.resize_discard(reporter_energy_offsets_counts.total_count());

        for(auto&& [i, R] : enumerate(reporter_view))
        {
            ComputeEnergyInfo this_info{side_streams};
            auto [offset, count] = reporter_energy_offsets_counts[i];
            this_info.m_energies = reporter_energies.view(offset, count);
            this_info.m_dt       = info.dt();
            R->compute_energy(this_info);
        }
    }
}

void ABDLineSearchReporter::Impl::reduce_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace cuda_tool;

    // Every launch goes to the reduction stream (after the producers).
    cudaStream_t stream = info.stream();

    // Zero out the kinetic energy of fixed bodies and bodies with external kinetic
    int n = (int)abd().abd_body_count;
    if(n > 0)
    {
        auto k = abd_line_search_reporter_compute_energy_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            abd().body_id_to_is_fixed.cview(),
            abd().body_id_to_external_kinetic.cview(),
            body_id_to_kinetic_energy.view(),
            n);
    }

    // Sum up the kinetic energy
    DeviceReduce(stream).Sum(body_id_to_kinetic_energy.data(),
                             abd_kinetic_energy.data(),
                             body_id_to_kinetic_energy.size());

    // Sum up the shape energy
    DeviceReduce(stream).Sum(body_id_to_shape_energy.data(),
                             abd_shape_energy.data(),
                             body_id_to_shape_energy.size());

    // Compute the total energy from all reporters
    DeviceReduce(stream).Sum(reporter_energies.data(),
                             total_reporter_energy.data(),
                             reporter_energies.size());

    abd_line_search_reporter_combine_energy_kernel<<<1, 1, 0, stream>>>(
        abd_kinetic_energy.cview(),
        abd_shape_energy.cview(),
        total_reporter_energy.cview(),
        info.energy());
}

void ABDLineSearchReporter::do_init(LineSearchReporter::InitInfo& info)
{
    m_impl.init(info);
}

void ABDLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void ABDLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void ABDLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void ABDLineSearchReporter::do_reduce_energy(LineSearcher::ComputeEnergyInfo& info)
{
    m_impl.reduce_energy(info);
}

void ABDLineSearchReporter::add_reporter(ABDLineSearchSubreporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is null");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.reporters.register_sim_system(*reporter);
}
}  // namespace uipc::backend::cuda
