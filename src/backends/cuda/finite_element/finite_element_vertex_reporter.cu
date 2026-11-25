/*
 * NVIDIA contributions: fused retrieval of the step and the vertex
 * displacements with their per-axis maximum, and batched start-point copies.
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

#include <finite_element/finite_element_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <finite_element/finite_element_body_reporter.h>
#include <uipc/builtin/attribute_name.h>
#include <global_geometry/axis_max.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementVertexReporter_init_attributes_kernel(
        cuda_tool::BufferView<IndexT>   coindices,
        cuda_tool::CBufferView<Vector3> src_pos,
        cuda_tool::BufferView<Vector3>  dst_pos,
        cuda_tool::CBufferView<Vector3> src_rest_pos,
        IndexT                          body_offset,
        cuda_tool::BufferView<IndexT>   dst_body_ids,
        cuda_tool::BufferView<Vector3>  dst_rest_pos,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i)    = i;
        dst_pos(i)      = src_pos(i);
        dst_rest_pos(i) = src_rest_pos(i);
        dst_body_ids(i) += body_offset;  // offset by the global body offset
    }
}  // namespace

REGISTER_SIM_SYSTEM(FiniteElementVertexReporter);

constexpr static U64 FiniteElementVertexReporterUID = 1;

void FiniteElementVertexReporter::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.body_reporter         = &require<FiniteElementBodyReporter>();
}

void FiniteElementVertexReporter::request_attribute_update() noexcept
{
    m_impl.require_update_attributes = true;
}

void FiniteElementVertexReporter::Impl::report_count(VertexCountInfo& info)
{
    info.count(fem().xs.size());
}

void FiniteElementVertexReporter::Impl::init_attributes(VertexAttributeInfo& info)
{
    info.contact_element_ids().copy_from(fem().h_vertex_contact_element_ids.data());
    info.subscene_element_ids().copy_from(
        fem().h_vertex_subscene_contact_element_ids.data());

    info.dimensions().copy_from(fem().h_dimensions.data());
    info.thicknesses().copy_from(fem().thicknesses);

    info.body_ids().copy_from(fem().h_vertex_body_id.data());
    info.d_hats().copy_from(fem().h_vertex_d_hat.data());

    // fill the coindices for later use
    auto k = FiniteElementVertexReporter_init_attributes_kernel;
    int  n = (int)info.coindices().size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.coindices(),
            fem().xs.cview(),
            info.positions(),
            fem().x_bars.cview(),
            body_reporter->body_offset(),
            info.body_ids(),
            info.rest_positions(),
            n);
    }
}

void FiniteElementVertexReporter::Impl::update_attributes(VertexAttributeInfo& info)
{
    info.positions().copy_from(fem().xs);

    // This update will ruin the friction force computed in previous step, so we need to discard it.
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    info.require_discard_friction();
}

namespace
{
    // A deferred retrieval and the displacements in one kernel (the step as
    // FEMLinearSubsystem_retrieve_solution_kernel, copied into the
    // displacements and folded into the per-axis maximum)
    __global__ void FiniteElementVertexReporter_retrieve_displacements_kernel(
        cuda_tool::BufferView<Vector3>     displacements,
        cuda_tool::BufferView<Vector3>     dxs,
        cuda_tool::CDenseVectorView<Float> x,
        int                                n,
        unsigned long long*                axis_max,
        unsigned long long*                axis_max_clear)
    {
        int        i      = blockIdx.x * blockDim.x + threadIdx.x;
        const bool active = i < n;
        Vector3    dx     = Vector3::Zero();
        if(active)
        {
            dx               = -x.segment<3>(i * 3).as_eigen();
            dxs(i)           = dx;
            displacements(i) = dx;
        }
        axis_max_add(axis_max, axis_max_clear, active, dx);
    }
}  // namespace

void FiniteElementVertexReporter::Impl::report_displacements(VertexDisplacementInfo& info)
{
    if(fem().solution_pending)
    {
        fem().solution_pending = false;
        auto k = FiniteElementVertexReporter_retrieve_displacements_kernel;
        int  n = (int)fem().dxs.size();
        if(n == 0)
        {
            info.zero_displacements();
            return;
        }
        auto a = info.axis_max();
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.displacements(), fem().dxs.view(), fem().pending_solution, n, a.slot, a.clear);
        }
        return;
    }
    info.copy_displacements_from(fem().dxs);
}

void FiniteElementVertexReporter::do_report_count(VertexCountInfo& info)
{
    m_impl.report_count(info);
}

void FiniteElementVertexReporter::do_report_attributes(VertexAttributeInfo& info)
{
    if(info.frame() == 0)
    {
        auto global_offset = info.coindices().offset();

        auto geo_slots = world().scene().geometries();

        // add global vertex offset attribute
        m_impl.finite_element_method->for_each(  //
            geo_slots,
            [&](const FiniteElementMethod::ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                auto gvo = sc.meta().find<IndexT>(builtin::global_vertex_offset);
                if(!gvo)
                {
                    gvo = sc.meta().create<IndexT>(builtin::global_vertex_offset);
                }

                // [global-vertex-offset] = [vertex-offset-in-fem-system] + [fem-system-vertex-offset]
                view(*gvo)[0] = I.geo_info().vertex_offset + global_offset;
            });

        m_impl.init_attributes(info);
    }
    else
    {
        if(m_impl.require_update_attributes)
        {
            m_impl.update_attributes(info);
            m_impl.require_update_attributes = false;
        }
    }
}

void FiniteElementVertexReporter::do_report_displacements(VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}

U64 FiniteElementVertexReporter::get_uid() const noexcept
{
    return FiniteElementVertexReporterUID;
}
}  // namespace uipc::backend::cuda
