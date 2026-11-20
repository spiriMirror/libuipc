/*
 * NVIDIA contributions: prelaunched FEM assembly beside the DyTopo work, the
 * gather beside the legacy stream, row-index-only triplet clears, the
 * shortened assembly tail and in-place writes into the global system; the
 * in-place prelaunch split around the contact launch, with the grid-capped
 * elastic producers issued right after the contact assembly kernels.
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
#include <sim_engine.h>
#include <finite_element/fem_linear_subsystem.h>
#include <finite_element/fem_linear_subsystem_reporter.h>
#include <finite_element/finite_element_kinetic.h>
#include <kernel_cout.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <finite_element/fem_dytopo_effect_receiver.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/flag.h>
#include <utils/report_extent_check.h>

namespace uipc::backend::cuda
{
// ============================================================================
// Named kernels (replacements for the former lambda kernel launches)
// ============================================================================
namespace
{
    __global__ void FEMLinearSubsystem_assemble_k1_kernel(cuda_tool::CBufferView<IndexT> is_fixed,
                                                          cuda_tool::DenseVectorView<Float> gradients,
                                                          int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(is_fixed(i))
        {
            gradients.segment<3>(i * 3).as_eigen().setZero();
        }
    }

    __global__ void FEMLinearSubsystem_assemble_k2_kernel(
        cuda_tool::CBufferView<IndexT>            is_fixed,
        cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
        int                                       n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, j, H3] = hessians(I).read();

        if(is_fixed(i) || is_fixed(j))
        {
            if(i != j)
                hessians(I).write(i, j, Matrix3x3::Zero());
            else
                hessians(I).write(i, j, Matrix3x3::Identity());
        }
    }

    __global__ void FEMLinearSubsystem_assemble_kinetic_kernel(
        cuda_tool::DenseVectorView<Float>       dst,
        cuda_tool::CDoubletVectorView<Float, 3> src,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, G3] = src(I);
        if(is_fixed(i))
            return;
        dst.segment<3>(i * 3).atomic_add(G3);
    }

    __global__ void FEMLinearSubsystem_assemble_reporters_kernel(
        cuda_tool::DenseVectorView<Float>       dst,
        cuda_tool::CDoubletVectorView<Float, 3> src,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, G3] = src(I);
        if(is_fixed(i))
            return;
        dst.segment<3>(i * 3).atomic_add(G3);
    }

    // Copy the prelaunched kinetic/reporter Hessians (FEM-local indices) into
    // the subsystem's range of the global system, applying on the way the
    // fixed-vertex rule of FEMLinearSubsystem_assemble_k2_kernel (which then
    // skips this range). The prelaunch marks every slot unwritten by its row
    // index alone; a slot no producer wrote becomes (-1, -1, 0), as the
    // global clear leaves it when the producers write the global system
    // directly. Grid-stride: the launch caps the grid at one block per SM,
    // so the legacy stream's gradient additions find room beside it.
    __global__ void FEMLinearSubsystem_gather_prelaunched_hessians_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> src,
        cuda_tool::TripletMatrixView<Float, 3, 3>  dst,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        int                                        n)
    {
        for(int I = blockIdx.x * blockDim.x + threadIdx.x; I < n;
            I += gridDim.x * blockDim.x)
        {
            const int row = src.row_indices()(I);
            if(row < 0)
            {
                dst.row_indices()(I) = -1;
                dst.col_indices()(I) = -1;
                dst.values()(I)      = Matrix3x3::Zero();
                continue;
            }
            auto&& [i, j, H3] = src(I);
            if(is_fixed(i) || is_fixed(j))
            {
                if(i != j)
                    dst(I).write(i, j, Matrix3x3::Zero());
                else
                    dst(I).write(i, j, Matrix3x3::Identity());
            }
            else
                dst(I).write(i, j, H3);
        }
    }

    // The gather's two rules applied in place to an in-place prelaunch's
    // prefix of the global system (global block indices; `offset` is the
    // subsystem's blocked DoF offset): a slot no producer wrote (row -1)
    // becomes (-1, -1, 0), a block touching a fixed vertex 0 (off-diagonal)
    // or I (diagonal).
    __global__ void FEMLinearSubsystem_fix_in_place_hessians_kernel(
        cuda_tool::BufferView<int>       rows,
        cuda_tool::BufferView<int>       cols,
        cuda_tool::BufferView<Matrix3x3> values,
        cuda_tool::CBufferView<IndexT>   is_fixed,
        int                              offset,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const int row = rows(I);
        if(row < 0)
        {
            cols(I)   = -1;
            values(I) = Matrix3x3::Zero();
            return;
        }
        const int i = row - offset;
        const int j = cols(I) - offset;
        if(is_fixed(i) || is_fixed(j))
        {
            if(i != j)
                values(I) = Matrix3x3::Zero();
            else
                values(I) = Matrix3x3::Identity();
        }
    }

    __global__ void FEMLinearSubsystem_assemble_dytopo_effect_k1_kernel(
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        cuda_tool::DenseVectorView<Float>       gradients,
        IndexT                                  vertex_offset,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, G3] = dytopo_effect_gradient(I);
        auto i                = g_i - vertex_offset;  // from global to local

        if(is_fixed(i))
            return;

        gradients.segment<3>(i * 3).atomic_add(G3);
    }

    __global__ void FEMLinearSubsystem_assemble_dytopo_effect_k2_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3, 3>  hessians,
        IndexT                                     vertex_offset,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, g_j, H3] = dytopo_effect_hessian(I);
        auto i                     = g_i - vertex_offset;
        auto j                     = g_j - vertex_offset;
        hessians(I).write(i, j, H3);
    }

    __global__ void FEMLinearSubsystem_retrieve_solution_kernel(
        cuda_tool::BufferView<Vector3> dxs, cuda_tool::CDenseVectorView<Float> result, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dxs(i) = -result.segment<3>(i * 3).as_eigen();
    }

    __global__ void FEMLinearSubsystem_diag_norm_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> triplet,
        cuda_tool::BufferView<Float>               diag_blocks_norm,
        IndexT                                     fem_segment_offset,
        IndexT                                     fem_segment_count,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [g_i, g_j, H3x3] = triplet(I);

        IndexT i = g_i - fem_segment_offset;
        IndexT j = g_j - fem_segment_offset;

        if(i >= fem_segment_count || j >= fem_segment_count)
            return;
        if(i == j)
        {
            auto a              = abs(H3x3(0, 0));
            auto b              = abs(H3x3(1, 1));
            auto c              = abs(H3x3(2, 2));
            diag_blocks_norm(i) = is_fixed(i) ? 0 : max(max(a, b), c);
        }
    }

    __global__ void FEMLinearSubsystem_mass_norm_kernel(cuda_tool::CBufferView<Float> mass,
                                                        cuda_tool::BufferView<Float> diag_blocks_norm,
                                                        cuda_tool::CBufferView<IndexT> is_fixed,
                                                        int n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        diag_blocks_norm(I) = is_fixed(I) ? 0 : mass(I);
    }
}  // namespace

REGISTER_SIM_SYSTEM(FEMLinearSubsystem);

FEMLinearSubsystem::~FEMLinearSubsystem()
{
    if(m_impl.gradients_cleared)
        cudaEventDestroy(m_impl.gradients_cleared);
}

// ref: https://github.com/spiriMirror/libuipc/issues/271
constexpr U64 FEMLinearSubsystemUID = 1ull;

U64 FEMLinearSubsystem::get_uid() const noexcept
{
    return FEMLinearSubsystemUID;
}

void FEMLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo&)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();
    m_impl.finite_element_vertex_reporter = require<FiniteElementVertexReporter>();
    m_impl.sim_engine = &engine();
    m_impl.dt_attr    = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");

    m_impl.dytopo_effect_receiver = find<FEMDyTopoEffectReceiver>();
}

void FEMLinearSubsystem::do_init(DiagLinearSubsystem::InitInfo& info)
{
    m_impl.init();
}

void FEMLinearSubsystem::Impl::init()
{
    auto reporter_view = reporters.view();
    for(auto&& [i, r] : enumerate(reporter_view))
        r->m_index = i;
    for(auto& r : reporter_view)
        r->init();

    reporter_gradient_offsets_counts.resize(reporter_view.size());
    reporter_hessian_offsets_counts.resize(reporter_view.size());

    int device = 0;
    CUDA_TOOL_CHECK(cudaGetDevice(&device));
    CUDA_TOOL_CHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, device));
}

void FEMLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(fem().xs.size() * 3);
    // the prelaunch writes the kinetic and reporter terms in place
    info.prelaunch_in_place();
}

void FEMLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = fem().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    finite_element_method->for_each(
        geo_slots,
        [&](const FiniteElementMethod::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on FEM mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on FEM mesh why can it happen?");

            IndexT this_dof_count = 3 * sc.vertices().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

void FEMLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    bool gradient_only = info.gradient_only();

    bool has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    // 1) Hessian Count
    IndexT grad_offset = 0;
    IndexT hess_offset = 0;

    // We assume reporters won't produce contact
    if(has_complement)
    {
        // Kinetic
        auto kinetic_grad_count = fem().xs.size();
        grad_offset += kinetic_grad_count;
        if(!gradient_only)
            hess_offset += fem().xs.size();

        // Reporters
        // [KineticG ... | OtherG ... ]
        // [KineticH ... | OtherH ... ]
        _report_reporter_extents(gradient_only);

        grad_offset += reporter_gradient_offsets_counts.total_count();
        hess_offset += reporter_hessian_offsets_counts.total_count();
    }

    if(dytopo_effect_receiver)  // if dytopo_effect enabled
    {
        grad_offset += dytopo_effect_receiver->gradients().doublet_count();
        hess_offset += dytopo_effect_receiver->hessians().triplet_count();

        UIPC_ASSERT(!(gradient_only
                      && !dytopo_effect_receiver->hessians().triplet_count() == 0),
                    "When gradient_only is true, hessian_offset must be 0, yours {}.\n"
                    "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                    dytopo_effect_receiver->hessians().triplet_count());
    }

    // 2) Gradient Count
    auto dof_count = fem().xs.size() * 3;

    UIPC_ASSERT(!(gradient_only && !hess_offset == 0),
                "When gradient_only is true, hessian_offset must be 0, yours {}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                hess_offset);

    info.extent(hess_offset, dof_count);
}

void FEMLinearSubsystem::Impl::_report_reporter_extents(bool gradient_only)
{
    auto grad_counts = reporter_gradient_offsets_counts.counts();
    auto hess_counts = reporter_hessian_offsets_counts.counts();

    for(auto& reporter : reporters.view())
    {
        ReportExtentInfo this_info;
        this_info.m_gradient_only = gradient_only;
        reporter->report_extent(this_info);
        grad_counts[reporter->m_index] = this_info.m_gradient_count;
        hess_counts[reporter->m_index] = this_info.m_hessian_count;

        UIPC_ASSERT(!(gradient_only && !this_info.m_hessian_count == 0),
                    "When gradient_only is true, hessian_offset must be 0, yours {}.\n"
                    "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                    this_info.m_hessian_count);
    }

    reporter_gradient_offsets_counts.scan();
    reporter_hessian_offsets_counts.scan();
}

void FEMLinearSubsystem::Impl::_prepare_gradient_buffers()
{
    kinetic_gradients.resize_doublets_discard(fem().xs.size());
    kinetic_gradients.reshape(fem().xs.size());
    loose_resize_entries(reporter_gradients, reporter_gradient_offsets_counts.total_count());
    reporter_gradients.reshape(fem().xs.size());
}

void FEMLinearSubsystem::Impl::prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info)
{
    // The point of the legacy stream before the DyTopo work: the prelaunched
    // producers need only the work enqueued so far (the Newton iterate).
    prelaunch_stream.mark();
}

void FEMLinearSubsystem::Impl::prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
    // Kinetic and reporter terms read only the Newton iterate and the
    // frame's animation targets, their extents do not depend on contact, and
    // they write FEM-private buffers here, so they run on a side stream
    // alongside the DyTopo work of the legacy stream. The light producers
    // (the FEM-local clear, the kinetic term, reporters that launch beside
    // the contact) start from the point prepare_prelaunch() marked, so they
    // run beside the contact assembly kernels; their host launches follow
    // the contact launches, so the contact blocks are placed first. The
    // other reporters wait for the contact assembly: an elastic producer
    // fills whole SMs with one block each and, dispatched beside the
    // contact kernels, would hold back the latency-bound contact kernels;
    // after them, it runs while the legacy stream distributes the contact
    // entries. Called right after the latency-bound contact launch
    // (ahead_of_contact(), from prelaunch_ahead()), the in-place path goes
    // first instead: the clears and the elastic producer, held to
    // elastic_block_cap() whole-SM blocks, run while the contact kernels hold
    // the other SMs; the light producers and the fix-up follow the rest of the
    // contact launch (the second call, from the DyTopo hook).
    if(ahead_pending)
    {
        ahead_pending = false;
        _prelaunch_in_place_rest(info);
        return;
    }
    prelaunched       = false;
    auto vertex_count = fem().xs.size();
    // Without vertices nothing is launched. With vertices the kinetic
    // Hessians make the system non-empty, so the next assemble() always
    // joins the prelaunch stream before the legacy stream moves on.
    if(vertex_count == 0)
        return;
    if(info.ahead_of_contact())
    {
        // Ahead only in place, and only when one round of whole-SM blocks
        // (256 threads, one element each) covers every 3D constitution's
        // producer (each launches on its own): within the cap it then takes
        // at most two rounds. A larger one would hold the contact kernels
        // back, so it waits for them.
        SizeT elements = 0;
        for(auto&& c : fem().fem_3d_constitution_infos)
            elements = std::max(elements, c.primitive_count);
        const SizeT elastic_blocks = (elements + 255) / 256;
        if(!info.in_place_available()
           || elastic_blocks > static_cast<SizeT>(info.sm_count()))
            return;
    }
    else if(!info.contact_launched())
        return;

    _report_reporter_extents(false);
    _prepare_gradient_buffers();
    prelaunched_gradient_count = reporter_gradients.doublet_count();
    prelaunched_hessian_count =
        vertex_count + reporter_hessian_offsets_counts.total_count();

    in_place = info.in_place_available();
    if(in_place)
    {
        if(info.ahead_of_contact())
            _prelaunch_in_place_ahead(info);
        else
            _prelaunch_in_place(info);
        return;
    }

    if(prelaunched_hessians.triplet_capacity() < prelaunched_hessian_count)
        prelaunched_hessians.reserve_triplets_discard(prelaunched_hessian_count * reserve_ratio);
    prelaunched_hessians.resize_triplets_discard(prelaunched_hessian_count);
    prelaunched_hessians.reshape(vertex_count, vertex_count);

    cudaStream_t stream = prelaunch_stream.fork_marked();

    // Producers may skip slots (the global clear leaves those as (-1, -1, 0)).
    // Marking every slot by its row index is enough: the gather writes a slot
    // whose row stayed -1 as (-1, -1, 0) and reads the column and value of
    // written slots only. Filling the values (the largest array, at the head
    // of this stream, in front of the elastic producer) and the column indices
    // as well is not needed.
    auto H = prelaunched_hessians.view();
    H.row_indices().fill(-1, stream);

    _compute_kinetic(false, H, stream);
    _compute_reporters(false, H, stream, ReporterSet::BesideContact);
    prelaunch_stream.fork();  // after the contact assembly kernels
    _compute_reporters(false, H, stream, ReporterSet::AfterContact);
    // assemble() gathers every slot of this range: the global clear skips it
    info.fully_written_triplets(prelaunched_hessian_count);
    prelaunched = true;
}

namespace
{
    // Checks the FEM-stream contract of the producers launched in between
    // (AssembleInfo::stream()): while a blocking stream captures, any use of
    // the legacy stream fails and invalidates the capture. Nothing is
    // captured: the producers launch on their non-blocking stream.
    class LegacyStreamCheck
    {
      public:
        LegacyStreamCheck(cudaStream_t blocking, bool on)
            : m_stream(on ? blocking : nullptr)
        {
            if(m_stream)
                CUDA_TOOL_CHECK(cudaStreamBeginCapture(m_stream, cudaStreamCaptureModeRelaxed));
        }
        // false: a producer used the legacy stream (its launch did not run)
        bool passed()
        {
            if(!m_stream)
                return true;
            cudaGraph_t graph = nullptr;
            cudaError_t e     = cudaStreamEndCapture(m_stream, &graph);
            if(graph)
                cudaGraphDestroy(graph);
            m_stream = nullptr;
            (void)cudaGetLastError();  // the failed launch's error
            return e == cudaSuccess;
        }

      private:
        cudaStream_t m_stream = nullptr;
    };
}  // namespace

void FEMLinearSubsystem::Impl::_prelaunch_in_place_ahead(GlobalLinearSystem::PrelaunchInfo& info)
{
    // As _prelaunch_in_place(), split around the contact launch: here the
    // clears and the after-contact reporters (the elastic producer, held to
    // the grid cap); the light producers start from the clears on a second
    // stream, issued after the contact kernels by _prelaunch_in_place_rest().
    if(!gradients_cleared)
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&gradients_cleared, cudaEventDisableTiming));
    if(!stream_check_stream)
        CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&stream_check_stream, cudaStreamDefault));

    ahead_hessians  = info.in_place_hessians(prelaunched_hessian_count);
    ahead_gradients = info.in_place_gradients();

    cudaStream_t stream = prelaunch_stream.fork_marked();
    ahead_hessians.row_indices().fill(-1, stream);
    ahead_gradients.buffer_view().fill(0.0, stream);
    CUDA_TOOL_CHECK(cudaEventRecord(gradients_cleared, stream));
    ahead_light = prelaunch_add_stream.fork(stream);

    ahead_check = stream_checked_hessians != prelaunched_hessian_count
                  || stream_checked_gradients != prelaunched_gradient_count;
    ahead_added_directly.assign(reporters.view().size(), 0);
    {
        LegacyStreamCheck guard{stream_check_stream, ahead_check};
        _compute_reporters(false,
                           ahead_hessians,
                           stream,
                           ReporterSet::AfterContact,
                           ahead_gradients,
                           &ahead_added_directly,
                           info.elastic_block_cap());
        UIPC_ASSERT(guard.passed(),
                    "A FEM producer launched outside AssembleInfo::stream() "
                    "(after-contact reporter)");
    }
    ahead_stream  = stream;
    ahead_pending = true;
}

void FEMLinearSubsystem::Impl::_prelaunch_in_place_rest(GlobalLinearSystem::PrelaunchInfo& info)
{
    auto&        H      = ahead_hessians;
    auto&        G      = ahead_gradients;
    cudaStream_t stream = ahead_stream;
    cudaStream_t light  = ahead_light;
    {
        LegacyStreamCheck guard{stream_check_stream, ahead_check};
        _compute_kinetic(false, H, light);
        _compute_reporters(false, H, light, ReporterSet::BesideContact);
        UIPC_ASSERT(guard.passed(),
                    "A FEM producer launched outside AssembleInfo::stream() "
                    "(kinetic or beside-contact reporter)");
    }
    stream_checked_hessians  = prelaunched_hessian_count;
    stream_checked_gradients = prelaunched_gradient_count;

    // kinetic and beside-contact gradients, after their producers
    {
        auto k = FEMLinearSubsystem_assemble_kinetic_kernel;
        int  n = (int)kinetic_gradients.doublet_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, light>>>(
                G, kinetic_gradients.cview(), fem().is_fixed.cview(), n);
    }
    for(auto& R : reporters.view())
        if(R->launches_beside_contact())
        {
            auto [offset, count] = reporter_gradient_offsets_counts[R->m_index];
            _add_reporter_gradients(G, offset, count, light);
        }

    // reporters that wrote doublets: add them here
    for(auto& R : reporters.view())
        if(!R->launches_beside_contact() && !ahead_added_directly[R->m_index])
        {
            auto [offset, count] = reporter_gradient_offsets_counts[R->m_index];
            _add_reporter_gradients(G, offset, count, stream);
        }

    prelaunch_add_stream.join(stream);
    {
        auto k = FEMLinearSubsystem_fix_in_place_hessians_kernel;
        int  n = (int)prelaunched_hessian_count;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
                H.row_indices(),
                H.col_indices(),
                H.values(),
                fem().is_fixed.cview(),
                info.in_place_block_offset(),
                n);
    }
    info.in_place_done(stream);

    // assemble() writes nothing of this range: the global clear skips it
    info.fully_written_triplets(prelaunched_hessian_count);
    prelaunched = true;
}

void FEMLinearSubsystem::Impl::_prelaunch_in_place(GlobalLinearSystem::PrelaunchInfo& info)
{
    // The producers write the subsystem's prefix of the global system and
    // its gradient range directly: no FEM-local buffer, no gather, no join
    // before the global assembly ends (GlobalLinearSystem waits for
    // in_place_done()). On the FEM stream: mark every slot unwritten, clear
    // the gradient range, the kinetic term and the beside-contact
    // reporters; after the contact assembly the other reporters (the
    // elastic producer adds its gradient directly) and the fix-up of the
    // gather's rules. The kinetic and beside-contact gradients are added on
    // a second stream forked after the contact assembly, beside the elastic
    // producer (before it, they would hold SMs from the contact kernels).
    if(!gradients_cleared)
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&gradients_cleared, cudaEventDisableTiming));
    if(!stream_check_stream)
        CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&stream_check_stream, cudaStreamDefault));

    auto H = info.in_place_hessians(prelaunched_hessian_count);
    auto G = info.in_place_gradients();

    cudaStream_t stream = prelaunch_stream.fork_marked();
    H.row_indices().fill(-1, stream);
    G.buffer_view().fill(0.0, stream);
    CUDA_TOOL_CHECK(cudaEventRecord(gradients_cleared, stream));

    // the stream contract is checked whenever the producers' extents change
    const bool check = stream_checked_hessians != prelaunched_hessian_count
                       || stream_checked_gradients != prelaunched_gradient_count;
    {
        LegacyStreamCheck guard{stream_check_stream, check};
        _compute_kinetic(false, H, stream);
        _compute_reporters(false, H, stream, ReporterSet::BesideContact);
        UIPC_ASSERT(guard.passed(),
                    "A FEM producer launched outside AssembleInfo::stream() "
                    "(kinetic or beside-contact reporter)");
    }

    // after the contact assembly kernels (ahead of them: right away)
    prelaunch_stream.fork();

    // kinetic and beside-contact gradients, beside the elastic producer
    cudaStream_t add = prelaunch_add_stream.fork(stream);
    {
        auto k = FEMLinearSubsystem_assemble_kinetic_kernel;
        int  n = (int)kinetic_gradients.doublet_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, add>>>(
                G, kinetic_gradients.cview(), fem().is_fixed.cview(), n);
    }
    for(auto& R : reporters.view())
        if(R->launches_beside_contact())
        {
            auto [offset, count] = reporter_gradient_offsets_counts[R->m_index];
            _add_reporter_gradients(G, offset, count, add);
        }

    vector<int> added_directly(reporters.view().size(), 0);
    {
        LegacyStreamCheck guard{stream_check_stream, check};
        _compute_reporters(false,
                           H,
                           stream,
                           ReporterSet::AfterContact,
                           G,
                           &added_directly,
                           info.ahead_of_contact() ? info.elastic_block_cap() : 0);
        UIPC_ASSERT(guard.passed(),
                    "A FEM producer launched outside AssembleInfo::stream() "
                    "(after-contact reporter)");
    }
    stream_checked_hessians  = prelaunched_hessian_count;
    stream_checked_gradients = prelaunched_gradient_count;

    // reporters that wrote doublets: add them here
    for(auto& R : reporters.view())
        if(!R->launches_beside_contact() && !added_directly[R->m_index])
        {
            auto [offset, count] = reporter_gradient_offsets_counts[R->m_index];
            _add_reporter_gradients(G, offset, count, stream);
        }

    {
        auto k = FEMLinearSubsystem_fix_in_place_hessians_kernel;
        int  n = (int)prelaunched_hessian_count;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
                H.row_indices(),
                H.col_indices(),
                H.values(),
                fem().is_fixed.cview(),
                info.in_place_block_offset(),
                n);
    }
    prelaunch_add_stream.join(stream);
    info.in_place_done(stream);

    // assemble() writes nothing of this range: the global clear skips it
    info.fully_written_triplets(prelaunched_hessian_count);
    prelaunched = true;
}

void FEMLinearSubsystem::Impl::_add_reporter_gradients(cuda_tool::DenseVectorView<Float> dst,
                                                       IndexT       offset,
                                                       IndexT       count,
                                                       cudaStream_t stream)
{
    auto k = FEMLinearSubsystem_assemble_reporters_kernel;
    if(count > 0)
        k<<<cuda_tool::best_grid_dim((int)count, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            dst,
            reporter_gradients.cview().subview(offset, count),
            fem().is_fixed.cview(),
            (int)count);
}

void FEMLinearSubsystem::Impl::_compute_kinetic(bool gradient_only,
                                                cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                                                cudaStream_t stream)
{
    IndexT kinetic_hess_count = gradient_only ? 0 : fem().xs.size();

    FEMLinearSubsystem::ComputeGradientHessianInfo kinetic_info{
        gradient_only,
        kinetic_gradients.view(),
        hessians.subview(0, kinetic_hess_count),
        dt_attr->view()[0],
        stream};
    kinetic->compute_gradient_hessian(kinetic_info);
}

void FEMLinearSubsystem::Impl::_compute_reporters(bool gradient_only,
                                                  cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                                                  cudaStream_t stream,
                                                  ReporterSet  set,
                                                  cuda_tool::DenseVectorView<Float> direct_gradients,
                                                  vector<int>* added_directly,
                                                  int          block_cap)
{
    IndexT kinetic_hess_count = gradient_only ? 0 : fem().xs.size();
    IndexT reporter_hess_count =
        gradient_only ? 0 : reporter_hessian_offsets_counts.total_count();

    auto reporter_hessian_view = hessians.subview(kinetic_hess_count, reporter_hess_count);
    for(auto& R : reporters.view())
    {
        if(set != ReporterSet::All
           && R->launches_beside_contact() != (set == ReporterSet::BesideContact))
            continue;
        AssembleInfo assemble_info{this, R->m_index, reporter_hessian_view, gradient_only, stream};
        assemble_info.m_direct_gradients = direct_gradients;
        assemble_info.m_block_cap        = block_cap;
        R->assemble(assemble_info);
        if(added_directly)
            (*added_directly)[R->m_index] = assemble_info.m_added_directly;
    }
}

void FEMLinearSubsystem::Impl::_assemble_kinetic_reporter_gradients(GlobalLinearSystem::DiagInfo& info)
{
    {
        auto k = FEMLinearSubsystem_assemble_kinetic_kernel;
        int  n = (int)kinetic_gradients.doublet_count();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.gradients(), kinetic_gradients.cview(), fem().is_fixed.cview(), n);
        }
    }

    {
        auto k = FEMLinearSubsystem_assemble_reporters_kernel;
        int  n = (int)reporter_gradients.doublet_count();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.gradients(), reporter_gradients.cview(), fem().is_fixed.cview(), n);
        }
    }
}

void FEMLinearSubsystem::Impl::_gather_prelaunched_hessians(IndexT& hess_offset,
                                                            GlobalLinearSystem::DiagInfo& info)
{
    // The legacy stream continues after the prelaunched producers (it adds
    // their gradients). The gather writes a range the global clear skipped
    // and FEMLinearSubsystem_assemble_k2_kernel no longer touches, so it runs
    // on the last assembly side stream, forked here and joined by the global
    // assembly before the BCOO conversion, beside the legacy stream's
    // gradient and DyTopo assembly.
    prelaunch_stream.join();

    cudaStream_t stream = nullptr;
    if(auto* side = info.side_streams(); side && side->size() > 0)
    {
        side->fork_one(side->size() - 1);
        stream = (*side)[side->size() - 1];
    }

    auto k = FEMLinearSubsystem_gather_prelaunched_hessians_kernel;
    int  n = (int)prelaunched_hessian_count;
    if(n > 0)
    {
        int grid = std::min(cuda_tool::best_grid_dim(n, k), sm_count);
        k<<<grid, cuda_tool::best_block_dim(k), 0, stream>>>(
            prelaunched_hessians.cview(),
            info.hessians().subview(hess_offset, n),
            fem().is_fixed.cview(),
            n);
    }
    hess_offset += n;
}

void FEMLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // 0) record dof info
    auto frame = sim_engine->frame();
    fem().set_dof_info(frame, info.gradients().offset(), info.gradients().size());

    bool has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    // A prelaunch serves exactly the next full assembly.
    bool use_prelaunch = prelaunched && info.prelaunched();
    // A discarded in-place prelaunch (the DoF layout changed): its producers
    // finish before this assembly reuses their buffers.
    if(prelaunched && !use_prelaunch && in_place)
        prelaunch_stream.join();
    UIPC_ASSERT(!use_prelaunch || (has_complement && !info.gradient_only()),
                "FEM prelaunch only serves a full assembly");

    // 1) Prepare Gradient Buffer (unchanged sizes after a prelaunch)
    _prepare_gradient_buffers();
    UIPC_ASSERT(!use_prelaunch
                    || (prelaunched_gradient_count == reporter_gradients.doublet_count()
                        && prelaunched_hessian_count
                               == fem().xs.size()
                                      + reporter_hessian_offsets_counts.total_count()),
                "FEM reporter extents changed after the prelaunch");

    // An in-place prelaunch cleared the gradient range on its stream and
    // added the kinetic and reporter gradients there.
    const bool use_in_place = use_prelaunch && in_place;
    if(!use_in_place)
        info.gradients().buffer_view().fill(0);

    // After a prelaunch, the DyTopo Hessians behind the gathered prefix and
    // the fixed-vertex rule of their range need only the DyTopo
    // distribution: they are written before the legacy stream joins the
    // prelaunched producers, so they run beside the elastic producer instead
    // of after it. The gradient keeps its order of additions.
    bool dytopo_hessians_written = false;
    if(use_prelaunch && dytopo_effect_receiver)
    {
        IndexT dytopo_offset = (IndexT)prelaunched_hessian_count;
        _assemble_dytopo_effect_hessians(dytopo_offset, info);
        _clear_fixed_vertex_hessians((IndexT)prelaunched_hessian_count, info);
        dytopo_hessians_written = true;
    }

    // 2) Assemble Gradient and Hessian
    IndexT hess_offset = 0;
    if(has_complement)
    {
        if(use_in_place)
        {
            hess_offset += (IndexT)prelaunched_hessian_count;
        }
        else if(use_prelaunch)
        {
            _gather_prelaunched_hessians(hess_offset, info);
        }
        else
        {
            IndexT hess_count =
                info.gradient_only() ?
                    0 :
                    fem().xs.size() + reporter_hessian_offsets_counts.total_count();
            _compute_kinetic(info.gradient_only(), info.hessians().subview(0, hess_count), nullptr);
            _compute_reporters(info.gradient_only(), info.hessians().subview(0, hess_count), nullptr);
            hess_offset += hess_count;
        }
        if(!use_in_place)
            _assemble_kinetic_reporter_gradients(info);
    }
    // the gathered prefix already holds the fixed-vertex blocks
    IndexT gathered_count = use_prelaunch ? (IndexT)prelaunched_hessian_count : 0;
    prelaunched = false;
    in_place    = false;

    if(dytopo_effect_receiver)  // if dytopo_effect enabled
    {
        // after the in-place prelaunch's clear of the range
        if(use_in_place)
            CUDA_TOOL_CHECK(cudaStreamWaitEvent(nullptr, gradients_cleared, 0));
        // DyTopo System will decide the `component_flags` itself
        _assemble_dytopo_effect_gradients(info);
        if(dytopo_hessians_written)
            hess_offset += (IndexT)dytopo_effect_receiver->hessians().triplet_count();
        else
            _assemble_dytopo_effect_hessians(hess_offset, info);
    }

    UIPC_ASSERT(hess_offset == info.hessians().triplet_count(),
                "Hessian offset mismatch expected {}, got {}",
                info.hessians().triplet_count(),
                hess_offset);


    // 3) Clear Fixed Vertex gradient (double check; after an in-place
    // prelaunch every addition skipped them already, and the prelaunched
    // gradients may still be in flight)
    if(!use_in_place)
    {
        auto k = FEMLinearSubsystem_assemble_k1_kernel;
        int  n = (int)fem().xs.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                fem().is_fixed.cview(), info.gradients(), n);
        }
    }

    if(info.gradient_only())
        return;

    // 4) Clear Fixed Vertex hessian
    if(!dytopo_hessians_written)
        _clear_fixed_vertex_hessians(gathered_count, info);
}

void FEMLinearSubsystem::Impl::_clear_fixed_vertex_hessians(IndexT begin,
                                                            GlobalLinearSystem::DiagInfo& info)
{
    auto k = FEMLinearSubsystem_assemble_k2_kernel;
    int  n = (int)info.hessians().triplet_count() - begin;
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            fem().is_fixed.cview(), info.hessians().subview(begin, n), n);
    }
}

void FEMLinearSubsystem::Impl::_assemble_dytopo_effect_gradients(GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // No need to add to grad_offset, the grad buffer is not from reporter_gradients
    auto grad_count = dytopo_effect_receiver->gradients().doublet_count();

    // Assemble DyTopoEffect Gradient to Gradient
    if(grad_count)
    {
        auto k = FEMLinearSubsystem_assemble_dytopo_effect_k1_kernel;
        k<<<cuda_tool::best_grid_dim((int)grad_count, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dytopo_effect_receiver->gradients(),
            info.gradients(),
            finite_element_vertex_reporter->vertex_offset(),
            fem().is_fixed.cview(),
            (int)grad_count);
    }
}

void FEMLinearSubsystem::Impl::_assemble_dytopo_effect_hessians(IndexT& hess_offset,
                                                                GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    if(info.gradient_only())
        return;

    // Need to update hess_offset, we are assembling to the global hessian buffer
    auto hess_count = dytopo_effect_receiver->hessians().triplet_count();

    // Assemble DyTopoEffect Hessian to Hessian
    if(hess_count)
    {
        auto dst_H3x3s = info.hessians().subview(hess_offset, hess_count);

        // NOTE: We don't consider fixed vertex here,
        // because in final phase we willclear the fixed vertex hessian anyway.
        auto k = FEMLinearSubsystem_assemble_dytopo_effect_k2_kernel;
        k<<<cuda_tool::best_grid_dim(hess_count, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dytopo_effect_receiver->hessians(),
            dst_H3x3s,
            finite_element_vertex_reporter->vertex_offset(),
            hess_count);
    }

    hess_offset += hess_count;
}

void FEMLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.satisfied(true);
}

void FEMLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace cuda_tool;

    auto dxs = fem().dxs.view();

    auto k = FEMLinearSubsystem_retrieve_solution_kernel;
    int  n = (int)fem().xs.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dxs, info.solution(), n);
    }
}

Float FEMLinearSubsystem::Impl::diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    diag_blocks_norm.resize(finite_element_method->xs().size());

    auto k = FEMLinearSubsystem_diag_norm_kernel;
    int  n = (int)info.A().triplet_count();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.A(),
            diag_blocks_norm.view(),
            info.dof_offset() / 3,
            info.dof_count() / 3,
            fem().is_fixed.cview(),
            n);
    }

    cuda_tool::DeviceReduce().Max(diag_blocks_norm.data(),
                                  reduced_diag_norm.data(),
                                  diag_blocks_norm.size());

    return reduced_diag_norm;
}

Float FEMLinearSubsystem::Impl::mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    diag_blocks_norm.resize(fem().xs.size());
    UIPC_ASSERT(fem().xs.size() == fem().masses.size(), "size not matched");

    auto k = FEMLinearSubsystem_mass_norm_kernel;
    int  n = (int)fem().xs.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            fem().masses.cview(), diag_blocks_norm.view(), fem().is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(diag_blocks_norm.data(),
                                  reduced_diag_norm.data(),
                                  diag_blocks_norm.size());

    return reduced_diag_norm;
}

void FEMLinearSubsystem::Impl::loose_resize_entries(cuda_tool::DeviceDoubletVector<Float, 3>& v,
                                                    SizeT size)
{
    if(size > v.doublet_capacity())
    {
        v.reserve_doublets_discard(size * reserve_ratio);
    }
    v.resize_doublets_discard(size);
}

void FEMLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void FEMLinearSubsystem::do_prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info)
{
    m_impl.prepare_prelaunch(info);
}

void FEMLinearSubsystem::do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
    m_impl.prelaunch_assembly(info);
}

void FEMLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void FEMLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void FEMLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

void FEMLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void FEMLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}

Float FEMLinearSubsystem::do_diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.diag_norm(info);
}

Float FEMLinearSubsystem::do_mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.mass_norm(info);
}

cuda_tool::DoubletVectorView<Float, 3> FEMLinearSubsystem::AssembleInfo::gradients() const
{
    auto [offset, count] = m_impl->reporter_gradient_offsets_counts[m_index];
    return m_impl->reporter_gradients.view().subview(offset, count);
}

cuda_tool::TripletMatrixView<Float, 3, 3> FEMLinearSubsystem::AssembleInfo::hessians() const
{
    auto [offset, count] = m_impl->reporter_hessian_offsets_counts[m_index];
    return m_hessians.subview(offset, count);
}

cuda_tool::CBufferView<IndexT> FEMLinearSubsystem::AssembleInfo::is_fixed() const noexcept
{
    return m_impl->fem().is_fixed.cview();
}

void FEMLinearSubsystem::AssembleInfo::add_gradients(cuda_tool::CDoubletVectorView<Float, 3> doublets) const
{
    UIPC_ASSERT(direct_gradients_available(), "no direct gradients to add to");
    auto k = FEMLinearSubsystem_assemble_reporters_kernel;
    int  n = (int)doublets.doublet_count();
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, m_stream>>>(
            m_direct_gradients, doublets, is_fixed(), n);
}

Float FEMLinearSubsystem::AssembleInfo::dt() const noexcept
{
    return m_impl->dt_attr->view()[0];
}

bool FEMLinearSubsystem::AssembleInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

void FEMLinearSubsystem::ReportExtentInfo::gradient_count(SizeT size)
{
    m_gradient_count = size;
}

void FEMLinearSubsystem::ReportExtentInfo::hessian_count(SizeT size)
{
    m_hessian_count = size;
}

void FEMLinearSubsystem::ReportExtentInfo::check(std::string_view name) const
{
    check_report_extent(m_gradient_only_checked, m_gradient_only, m_hessian_count, name);
}

void FEMLinearSubsystem::add_reporter(FEMLinearSubsystemReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter cannot be null");
    check_state(SimEngineState::BuildSystems, "add_reporter");
    m_impl.reporters.register_sim_system(*reporter);
}

void FEMLinearSubsystem::add_kinetic(FiniteElementKinetic* kinetic)
{
    UIPC_ASSERT(kinetic, "kinetic cannot be null");
    check_state(SimEngineState::BuildSystems, "add_kinetic");
    m_impl.kinetic.register_sim_system(*kinetic);
}
}  // namespace uipc::backend::cuda
