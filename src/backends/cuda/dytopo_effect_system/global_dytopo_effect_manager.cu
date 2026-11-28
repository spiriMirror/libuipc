/*
 * NVIDIA contributions: distribution of raw entries to several receivers,
 * side-stream reporter assembly, batched count read-backs and the prelaunch
 * hook; the packed selection, scan and fill of all receivers' entries in one
 * pass per entry kind with the gradient selection on a side stream, reporters
 * with one-warp assembly blocks launched first with a hook after them, and
 * their total block count.
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
#include <cstdint>
#include <numeric>
#include <sim_engine.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <dytopo_effect_system/dytopo_effect_reporter.h>
#include <dytopo_effect_system/dytopo_effect_receiver.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>
#include <energy_component_flags.h>
#include <cuda_tool/cub.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalDyTopoEffectManager>
{
  public:
    static U<cuda::GlobalDyTopoEffectManager> create(cuda::SimEngine& engine)
    {
        auto dytopo_effect_enable_attr =
            engine.world().scene().config().find<IndexT>("contact/enable");
        bool dytopo_effect_enable = dytopo_effect_enable_attr->view()[0] != 0;

        auto& types = engine.world().scene().constitution_tabular().types();
        bool  has_inter_primitive_constitution =
            types.find(std::string{builtin::InterPrimitive}) != types.end();

        if(dytopo_effect_enable || has_inter_primitive_constitution)
            return make_unique<cuda::GlobalDyTopoEffectManager>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
namespace
{
    // The receivers accept duplicate entries, so the distribution selects the
    // raw assembled doublets/triplets of each receiver's range in any order.

    // The selected counts (the last element of each exclusive scan) into
    // one array, so one copy reads them back; a null source gives 0.
    constexpr int MaxGatheredCounts = 16;
    struct GatheredCountSources
    {
        const IndexT* src[MaxGatheredCounts];
    };
    __global__ void GlobalDyTopoEffectManager_gather_counts_kernel(GatheredCountSources sources,
                                                                   IndexT* counts,
                                                                   int n)
    {
        int i = threadIdx.x;
        if(i < n)
            counts[i] = sources.src[i] ? *sources.src[i] : 0;
    }

    __global__ void GlobalDyTopoEffectManager_distribute_k1_kernel(
        cuda_tool::BufferView<IndexT>           selected_gradient,
        cuda_tool::Dense<IndexT>                last,
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        Vector2i                                range,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, G] = dytopo_effect_gradient(I);

        selected_gradient(I) = i >= range.x() && i < range.y() ? 1 : 0;

        // fill the last one as 0, so that we can calculate the total count
        // during the exclusive scan
        if(I == 0)
            last = 0;
    }

    __global__ void GlobalDyTopoEffectManager_distribute_k2_kernel(
        cuda_tool::CBufferView<IndexT>          selected_gradient,
        cuda_tool::CBufferView<IndexT>          selected_gradient_offsets,
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        cuda_tool::DoubletVectorView<Float, 3>  classified_gradient,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        if(selected_gradient(I))
        {
            auto&& [i, G] = dytopo_effect_gradient(I);
            classified_gradient(selected_gradient_offsets(I)).write(i, G);
        }
    }

    __global__ void GlobalDyTopoEffectManager_distribute_k3_kernel(
        cuda_tool::BufferView<IndexT>              selected_hessian,
        cuda_tool::Dense<IndexT>                   last,
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        Vector2i                                   i_range,
        Vector2i                                   j_range,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, j, H] = dytopo_effect_hessian(I);

        auto in_range = [](int i, const Vector2i& range)
        { return i >= range.x() && i < range.y(); };

        selected_hessian(I) = in_range(i, i_range) && in_range(j, j_range) ? 1 : 0;

        // fill the last one as 0, so that we can calculate the total count
        // during the exclusive scan
        if(I == 0)
            last = 0;
    }

    __global__ void GlobalDyTopoEffectManager_distribute_k4_kernel(
        cuda_tool::CBufferView<IndexT>             selected_hessian,
        cuda_tool::CBufferView<IndexT>             selected_hessian_offsets,
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3, 3>  classified_hessian,
        Vector2i                                   i_range,
        Vector2i                                   j_range,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        if(selected_hessian(I))
        {
            auto&& [i, j, H] = dytopo_effect_hessian(I);
            auto offset      = selected_hessian_offsets(I);

            classified_hessian(offset).write(i, j, H);
        }
    }

    // The selections of up to four receivers in one pass, packed into 64-bit
    // words (a field of 64 / slots bits per receiver), so that one scan gives
    // every receiver's offsets: the same integers as the per-receiver scans.
    constexpr int MaxPackedSlots = 4;
    struct PackedSelection
    {
        Vector2i i_range[MaxPackedSlots];
        Vector2i j_range[MaxPackedSlots];
        int      slots = 0;
        int      bits  = 64;
    };

    __device__ inline bool packed_in(int i, const Vector2i& r)
    {
        return i >= r.x() && i < r.y();
    }

    __global__ void GlobalDyTopoEffectManager_select_gradient_packed_kernel(
        cuda_tool::BufferView<uint64_t>         packed,
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        PackedSelection                         sel,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, G] = dytopo_effect_gradient(I);
        uint64_t v    = 0;
        for(int s = 0; s < sel.slots; ++s)
            if(packed_in(i, sel.i_range[s]))
                v |= uint64_t(1) << (sel.bits * s);
        packed(I) = v;
        if(I == 0)
            packed(n) = 0;
    }

    __global__ void GlobalDyTopoEffectManager_select_hessian_packed_kernel(
        cuda_tool::BufferView<uint64_t>            packed,
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        PackedSelection                            sel,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [i, j, H] = dytopo_effect_hessian(I);
        uint64_t v       = 0;
        for(int s = 0; s < sel.slots; ++s)
            if(packed_in(i, sel.i_range[s]) && packed_in(j, sel.j_range[s]))
                v |= uint64_t(1) << (sel.bits * s);
        packed(I) = v;
        if(I == 0)
            packed(n) = 0;
    }

    __device__ inline IndexT packed_field(uint64_t v, int shift, int bits)
    {
        return static_cast<IndexT>(
            (bits == 64 ? v : (v >> shift) & ((uint64_t(1) << bits) - 1)));
    }

    // The selected entries of every receiver in one pass: the receiver of
    // slot s writes at the offset in its field of the packed scan.
    struct PackedGradientTargets
    {
        cuda_tool::DoubletVectorView<Float, 3> to[MaxPackedSlots];
    };
    struct PackedHessianTargets
    {
        cuda_tool::TripletMatrixView<Float, 3, 3> to[MaxPackedSlots];
    };

    __global__ void GlobalDyTopoEffectManager_fill_gradient_packed_kernel(
        cuda_tool::CBufferView<uint64_t>        packed,
        cuda_tool::CBufferView<uint64_t>        offsets,
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        PackedGradientTargets                   classified_gradients,
        int                                     slots,
        int                                     bits,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const uint64_t v = packed(I);
        for(int s = 0; s < slots; ++s)
            if(packed_field(v, bits * s, bits))
            {
                auto&& [i, G] = dytopo_effect_gradient(I);
                classified_gradients
                    .to[s](packed_field(offsets(I), bits * s, bits))
                    .write(i, G);
            }
    }

    __global__ void GlobalDyTopoEffectManager_fill_hessian_packed_kernel(
        cuda_tool::CBufferView<uint64_t>           packed,
        cuda_tool::CBufferView<uint64_t>           offsets,
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        PackedHessianTargets                       classified_hessians,
        int                                        slots,
        int                                        bits,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const uint64_t v = packed(I);
        for(int s = 0; s < slots; ++s)
            if(packed_field(v, bits * s, bits))
            {
                auto&& [i, j, H] = dytopo_effect_hessian(I);
                classified_hessians
                    .to[s](packed_field(offsets(I), bits * s, bits))
                    .write(i, j, H);
            }
    }

    // counts[2r] / counts[2r + 1]: receiver r's gradient / Hessian count from
    // the packed totals (slot -1: none)
    struct PackedCountSlots
    {
        int g[MaxGatheredCounts / 2];
        int h[MaxGatheredCounts / 2];
    };
    __global__ void GlobalDyTopoEffectManager_gather_packed_counts_kernel(
        const uint64_t*  g_total,
        int              g_bits,
        const uint64_t*  h_total,
        int              h_bits,
        PackedCountSlots slots,
        IndexT*          counts,
        int              receivers)
    {
        int r = threadIdx.x;
        if(r >= receivers)
            return;
        counts[2 * r] =
            slots.g[r] < 0 ? 0 : packed_field(*g_total, g_bits * slots.g[r], g_bits);
        counts[2 * r + 1] =
            slots.h[r] < 0 ? 0 : packed_field(*h_total, h_bits * slots.h[r], h_bits);
    }
}  // namespace

REGISTER_SIM_SYSTEM(GlobalDyTopoEffectManager);

GlobalDyTopoEffectManager::~GlobalDyTopoEffectManager()
{
    if(m_impl.gradient_selected)
        cudaEventDestroy(m_impl.gradient_selected);
}

void GlobalDyTopoEffectManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_vertex_manager = require<GlobalVertexManager>();
}

void GlobalDyTopoEffectManager::Impl::init(WorldVisitor& world)
{
    // 3) reporters
    auto dytopo_effect_reporter_view = dytopo_effect_reporters.view();
    for(auto&& [i, R] : enumerate(dytopo_effect_reporter_view))
        R->init();
    for(auto&& [i, R] : enumerate(dytopo_effect_reporter_view))
        R->m_index = i;

    reporter_energy_offsets_counts.resize(dytopo_effect_reporter_view.size());
    reporter_gradient_offsets_counts.resize(dytopo_effect_reporter_view.size());
    reporter_hessian_offsets_counts.resize(dytopo_effect_reporter_view.size());

    // Normal and frictional contact of simplices and half planes: four
    // reporters in a typical scene, each on its own stream.
    constexpr SizeT max_assembly_stream_count = 4;
    assembly_streams.create(std::min<SizeT>(dytopo_effect_reporter_view.size(),
                                            max_assembly_stream_count));

    // 4) receivers
    auto dytopo_effect_receiver_view = dytopo_effect_receivers.view();
    for(auto&& [i, R] : enumerate(dytopo_effect_receiver_view))
        R->init();
    for(auto&& [i, R] : enumerate(dytopo_effect_receiver_view))
        R->m_index = i;

    classified_dytopo_effect_gradients.resize(dytopo_effect_receiver_view.size());
    classified_dytopo_effect_hessians.resize(dytopo_effect_receiver_view.size());
}

void GlobalDyTopoEffectManager::Impl::compute_dytopo_effect(ComputeDyTopoEffectInfo& info)
{
    _assemble(info);
    if(info.m_after_assembly_launch)
        info.m_after_assembly_launch();
    auto receivers = dytopo_effect_receivers.view();
    if(receivers.size() == 1)
    {
        DyTopoClassifyInfo classify_info;
        auto*              receiver = receivers[0];
        receiver->report(classify_info);

        auto dynamic_range = classify_info.gradient_i_range();
        auto vertex_count =
            static_cast<IndexT>(global_vertex_manager->positions().size());

        // A single diagonal receiver starting at zero owns the complete dynamic
        // vertex prefix. Any trailing global vertices are non-DOF geometry (for
        // example, half planes), whose contact kernels only differentiate the
        // dynamic vertex. Every raw entry therefore belongs to this receiver,
        // and selecting its range would only copy them.
        if(classify_info.is_diag() && dynamic_range.x() == 0 && dynamic_range.y() != 0
           && dynamic_range.y() <= vertex_count && classify_info.hessian_i_range() == dynamic_range
           && classify_info.hessian_j_range() == dynamic_range)
        {
            Timer timer{"Distribute Dytopo Effect"};

            ClassifiedDyTopoEffectInfo classified_info;
            classified_info.m_gradients = collected_dytopo_effect_gradient.cview();
            if(!info.m_gradient_only)
                classified_info.m_hessians = collected_dytopo_effect_hessian.cview();
            receiver->receive(classified_info);
            return;
        }
    }

    _distribute(info);
}

void GlobalDyTopoEffectManager::Impl::_assemble(ComputeDyTopoEffectInfo& info)
{
    Timer timer{"Assemble Dytopo Effect"};

    auto vertex_count = global_vertex_manager->positions().size();

    auto reporter_gradient_counts = reporter_gradient_offsets_counts.counts();
    auto reporter_hessian_counts  = reporter_hessian_offsets_counts.counts();
    bool gradient_only            = info.m_gradient_only;

    logger::info("DyTopo Effect Assembly: GradientOnly={}, ComponentFlags={}",
                 info.m_gradient_only,
                 enum_flags_name(info.m_component_flags));

    {
        Timer timer{"Report Extent"};
        for(auto&& [i, reporter] : enumerate(dytopo_effect_reporters.view()))
        {
            reporter_gradient_counts[i] = 0;
            reporter_hessian_counts[i]  = 0;

            if(!has_flags(info.m_component_flags, reporter->component_flags()))
                continue;

            GradientHessianExtentInfo extent_info;
            extent_info.m_gradient_only = gradient_only;
            reporter->report_gradient_hessian_extent(extent_info);

            reporter_gradient_counts[i] = extent_info.m_gradient_count;
            reporter_hessian_counts[i] = gradient_only ? 0 : extent_info.m_hessian_count;
            logger::info("<{}> DyTopo Grad3 count: {}, DyTopo Hess3x3 count: {}",
                         reporter->name(),
                         extent_info.m_gradient_count,
                         extent_info.m_hessian_count);
        }
    }

    {
        Timer timer{"Scan and Allocate"};
        // scan
        reporter_gradient_offsets_counts.scan();
        reporter_hessian_offsets_counts.scan();

        auto total_gradient_count = reporter_gradient_offsets_counts.total_count();
        auto total_hessian_count = reporter_hessian_offsets_counts.total_count();

        // allocate
        loose_resize_entries(collected_dytopo_effect_gradient, total_gradient_count);
        loose_resize_entries(collected_dytopo_effect_hessian, total_hessian_count);
        collected_dytopo_effect_gradient.reshape(vertex_count);
        collected_dytopo_effect_hessian.reshape(vertex_count, vertex_count);
    }

    // collect: reporters write disjoint ranges, each on its own side stream.
    // Those with latency-bound assembly kernels (report_assembly_blocks())
    // launch first: they lead the critical path.
    auto               reporters = dytopo_effect_reporters.view();
    std::vector<SizeT> launch_order(reporters.size());
    std::iota(launch_order.begin(), launch_order.end(), SizeT{0});
    const SizeT leading =
        std::ranges::stable_partition(
            launch_order,
            [&](SizeT i) { return reporters[i]->report_assembly_blocks() > 0; })
            .begin()
        - launch_order.begin();
    assembly_streams.fork();
    for(SizeT k = 0; k < launch_order.size(); ++k)
    {
        if(k == leading && info.m_after_leading_launch)
            info.m_after_leading_launch();
        const SizeT i        = launch_order[k];
        auto        reporter = reporters[i];
        if(!has_flags(info.m_component_flags, reporter->component_flags()))
            continue;

        auto [g_offset, g_count] = reporter_gradient_offsets_counts[i];
        auto [h_offset, h_count] = reporter_hessian_offsets_counts[i];

        GradientHessianInfo info;
        info.m_gradient_only = gradient_only;
        info.m_stream        = assembly_streams.next();

        info.m_gradients =
            collected_dytopo_effect_gradient.view().subview(g_offset, g_count);
        info.m_hessians = collected_dytopo_effect_hessian.view().subview(h_offset, h_count);

        reporter->assemble(info);
    }
    if(leading == launch_order.size() && info.m_after_leading_launch)
        info.m_after_leading_launch();
    // the distribution reads every reporter's range
    assembly_streams.join();
}

bool GlobalDyTopoEffectManager::Impl::_distribute_packed(ComputeDyTopoEffectInfo& info)
{
    using namespace cuda_tool;

    auto       vertex_count = global_vertex_manager->positions().size();
    auto       receivers    = dytopo_effect_receivers.view();
    const auto NG           = collected_dytopo_effect_gradient.doublet_count();
    const auto NH           = collected_dytopo_effect_hessian.triplet_count();
    if(2 * receivers.size() > MaxGatheredCounts)
        return false;

    PackedSelection  gsel, hsel;
    PackedCountSlots slots;
    receiver_classify_infos.resize(receivers.size());
    for(auto&& [i, receiver] : enumerate(receivers))
    {
        auto& classify_info = receiver_classify_infos[i];
        classify_info       = DyTopoClassifyInfo{};
        receiver->report(classify_info);
        slots.g[i] = -1;
        slots.h[i] = -1;
        if(classify_info.is_diag())
        {
            if(gsel.slots == MaxPackedSlots)
                return false;
            slots.g[i]                 = gsel.slots;
            gsel.i_range[gsel.slots++] = classify_info.gradient_i_range();
        }
        if(!info.m_gradient_only && !classify_info.is_empty())
        {
            if(hsel.slots == MaxPackedSlots)
                return false;
            slots.h[i]               = hsel.slots;
            hsel.i_range[hsel.slots] = classify_info.hessian_i_range();
            hsel.j_range[hsel.slots] = classify_info.hessian_j_range();
            ++hsel.slots;
        }
    }
    auto fits = [](PackedSelection& sel, SizeT n)
    {
        sel.bits = sel.slots > 0 ? 64 / sel.slots : 64;
        return sel.bits == 64 || n + 1 < (SizeT(1) << sel.bits);
    };
    if(!fits(gsel, NG) || !fits(hsel, NH))
        return false;

    auto select = [&](auto                    k,
                      DeviceBuffer<uint64_t>& packed,
                      DeviceBuffer<uint64_t>& offsets,
                      auto                    entries,
                      PackedSelection&        sel,
                      SizeT                   N,
                      cudaStream_t            stream)
    {
        loose_resize(packed, N + 1);
        loose_resize(offsets, N + 1);
        int n = (int)N;
        if(n > 0)
            k<<<best_grid_dim(n, k), best_block_dim(k), 0, stream>>>(
                packed.view(), entries, sel, n);
        // the scan reads the extra last entry too; its exclusive sum (the
        // total at offsets[N]) does not depend on it, but it must be written
        CUDA_TOOL_CHECK(cudaMemsetAsync(packed.data() + N, 0, sizeof(uint64_t), stream));
        DeviceScan(stream).ExclusiveSum(packed.data(), offsets.data(), n + 1);
    };
    // The two selections are independent integer passes over the collected
    // entries: the gradient one runs on a side stream beside the Hessian one,
    // and the legacy stream waits for it before it gathers the counts.
    cudaStream_t gradient_stream = nullptr;
    if(gsel.slots > 0 && hsel.slots > 0 && assembly_streams.size() > 0)
    {
        gradient_stream = assembly_streams[0];
        assembly_streams.fork_one(0);
    }
    if(gsel.slots > 0)
        select(GlobalDyTopoEffectManager_select_gradient_packed_kernel,
               packed_gradient,
               packed_gradient_offsets,
               std::as_const(collected_dytopo_effect_gradient).viewer(),
               gsel,
               NG,
               gradient_stream);
    if(hsel.slots > 0)
        select(GlobalDyTopoEffectManager_select_hessian_packed_kernel,
               packed_hessian,
               packed_hessian_offsets,
               collected_dytopo_effect_hessian.cviewer(),
               hsel,
               NH,
               nullptr);
    if(gradient_stream)
    {
        if(!gradient_selected)
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&gradient_selected, cudaEventDisableTiming));
        CUDA_TOOL_CHECK(cudaEventRecord(gradient_selected, gradient_stream));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(nullptr, gradient_selected, 0));
    }

    receiver_selected_counts.resize_discard(2 * receivers.size());
    if(gsel.slots > 0 || hsel.slots > 0)
    {
        GlobalDyTopoEffectManager_gather_packed_counts_kernel<<<1, MaxGatheredCounts / 2>>>(
            gsel.slots > 0 ? packed_gradient_offsets.data() + NG : nullptr,
            gsel.bits,
            hsel.slots > 0 ? packed_hessian_offsets.data() + NH : nullptr,
            hsel.bits,
            slots,
            receiver_selected_counts.device_data(),
            (int)receivers.size());
        receiver_selected_counts.enqueue_copy();
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    }

    // size every receiver's buffers, fill them all in one pass per kind
    PackedGradientTargets gtargets;
    PackedHessianTargets  htargets;
    bool                  any_gradient = false, any_hessian = false;
    for(auto&& [i, receiver] : enumerate(receivers))
    {
        const auto& classify_info  = receiver_classify_infos[i];
        auto& classified_gradients = classified_dytopo_effect_gradients[i];
        classified_gradients.reshape(vertex_count);
        auto& classified_hessians = classified_dytopo_effect_hessians[i];
        classified_hessians.reshape(vertex_count, vertex_count);

        if(classify_info.is_diag())
        {
            IndexT count = receiver_selected_counts[2 * i];
            loose_resize_entries(classified_gradients, count);
            gtargets.to[slots.g[i]] = classified_gradients.viewer();
            any_gradient |= count > 0;
        }
        if(!info.m_gradient_only && !classify_info.is_empty())
        {
            IndexT count = receiver_selected_counts[2 * i + 1];
            loose_resize_entries(classified_hessians, count);
            htargets.to[slots.h[i]] = classified_hessians.viewer();
            any_hessian |= count > 0;
        }
    }
    if(any_gradient)
    {
        auto k = GlobalDyTopoEffectManager_fill_gradient_packed_kernel;
        int  n = (int)NG;
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            packed_gradient.cview(),
            packed_gradient_offsets.cview(),
            std::as_const(collected_dytopo_effect_gradient).viewer(),
            gtargets,
            gsel.slots,
            gsel.bits,
            n);
    }
    if(any_hessian)
    {
        auto k = GlobalDyTopoEffectManager_fill_hessian_packed_kernel;
        int  n = (int)NH;
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            packed_hessian.cview(),
            packed_hessian_offsets.cview(),
            collected_dytopo_effect_hessian.cviewer(),
            htargets,
            hsel.slots,
            hsel.bits,
            n);
    }

    for(auto&& [i, receiver] : enumerate(receivers))
    {
        const auto& classify_info = receiver_classify_infos[i];

        ClassifiedDyTopoEffectInfo classified_info;
        if(classify_info.is_diag())
            classified_info.m_gradients = classified_dytopo_effect_gradients[i].view();
        if(!info.m_gradient_only && !classify_info.is_empty())
            classified_info.m_hessians = classified_dytopo_effect_hessians[i].view();
        receiver->receive(classified_info);
    }
    return true;
}

void GlobalDyTopoEffectManager::Impl::_distribute(ComputeDyTopoEffectInfo& info)
{
    Timer timer{"Distribute Dytopo Effect"};

    using namespace cuda_tool;

    // One pass and one scan per entry kind for all receivers when their
    // selections fit packed 64-bit words; otherwise one per receiver below.
    if(_distribute_packed(info))
        return;

    auto vertex_count = global_vertex_manager->positions().size();
    auto receivers    = dytopo_effect_receivers.view();

    const auto NG = collected_dytopo_effect_gradient.doublet_count();
    const auto NH = collected_dytopo_effect_hessian.triplet_count();

    receiver_selections.resize(receivers.size());
    receiver_classify_infos.resize(receivers.size());
    receiver_selected_counts.resize_discard(2 * receivers.size());
    const int            count_n = (int)(2 * receivers.size());
    const bool           gather  = count_n <= MaxGatheredCounts;
    GatheredCountSources sources = {};

    // 1) Select every receiver's entries first. Each exclusive scan ends with
    //    the selected count, and all counts come back in one host
    //    synchronization.
    bool has_counts = false;
    for(auto&& [i, receiver] : enumerate(receivers))
    {
        auto& classify_info = receiver_classify_infos[i];
        classify_info       = DyTopoClassifyInfo{};
        receiver->report(classify_info);

        auto& selection = receiver_selections[i];

        // gradient
        if(classify_info.is_diag())
        {
            // +1 for calculate the total count
            loose_resize(selection.gradient, NG + 1);
            loose_resize(selection.gradient_offsets, NG + 1);

            // select
            {
                auto k = GlobalDyTopoEffectManager_distribute_k1_kernel;
                int  n = (int)NG;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        selection.gradient.view(0, NG).viewer(),
                        VarView<IndexT>{selection.gradient.data() + NG}.viewer(),
                        std::as_const(collected_dytopo_effect_gradient).viewer(),
                        classify_info.gradient_i_range(),
                        n);
                }
            }

            // scan
            DeviceScan().ExclusiveSum(selection.gradient.data(),
                                      selection.gradient_offsets.data(),
                                      selection.gradient.size());

            if(gather)
                sources.src[2 * i] = selection.gradient_offsets.data() + NG;
            else
                receiver_selected_counts.copy_from(
                    2 * i, selection.gradient_offsets.cview(NG, 1));
            has_counts = true;
        }

        // hessian
        if(!info.m_gradient_only && !classify_info.is_empty())
        {
            // +1 for calculate the total count
            loose_resize(selection.hessian, NH + 1);
            loose_resize(selection.hessian_offsets, NH + 1);

            // select
            {
                auto k = GlobalDyTopoEffectManager_distribute_k3_kernel;
                int  n = (int)NH;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        selection.hessian.view(0, NH).viewer(),
                        VarView<IndexT>{selection.hessian.data() + NH}.viewer(),
                        collected_dytopo_effect_hessian.cviewer(),
                        classify_info.hessian_i_range(),
                        classify_info.hessian_j_range(),
                        n);
                }
            }

            // scan
            DeviceScan().ExclusiveSum(selection.hessian.data(),
                                      selection.hessian_offsets.data(),
                                      selection.hessian.size());

            if(gather)
                sources.src[2 * i + 1] = selection.hessian_offsets.data() + NH;
            else
                receiver_selected_counts.copy_from(
                    2 * i + 1, selection.hessian_offsets.cview(NH, 1));
            has_counts = true;
        }
    }

    if(has_counts)
    {
        if(gather)
        {
            GlobalDyTopoEffectManager_gather_counts_kernel<<<1, MaxGatheredCounts>>>(
                sources, receiver_selected_counts.device_data(), count_n);
            receiver_selected_counts.enqueue_copy();
        }
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    }

    // 2) Copy the selected entries and hand them to the receivers.
    for(auto&& [i, receiver] : enumerate(receivers))
    {
        const auto& classify_info = receiver_classify_infos[i];
        auto&       selection     = receiver_selections[i];

        ClassifiedDyTopoEffectInfo classified_info;
        auto& classified_gradients = classified_dytopo_effect_gradients[i];
        classified_gradients.reshape(vertex_count);
        auto& classified_hessians = classified_dytopo_effect_hessians[i];
        classified_hessians.reshape(vertex_count, vertex_count);

        // gradient
        if(classify_info.is_diag())
        {
            IndexT count = receiver_selected_counts[2 * i];

            loose_resize_entries(classified_gradients, count);

            // fill
            if(count > 0)
            {
                auto k = GlobalDyTopoEffectManager_distribute_k2_kernel;
                int  n = (int)NG;
                k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                    selection.gradient.cview(),
                    selection.gradient_offsets.cview(),
                    std::as_const(collected_dytopo_effect_gradient).viewer(),
                    classified_gradients.viewer(),
                    n);
            }

            classified_info.m_gradients = classified_gradients.view();
        }

        // hessian
        if(!info.m_gradient_only && !classify_info.is_empty())
        {
            IndexT h_total_count = receiver_selected_counts[2 * i + 1];

            loose_resize_entries(classified_hessians, h_total_count);

            // fill
            if(h_total_count > 0)
            {
                auto k = GlobalDyTopoEffectManager_distribute_k4_kernel;
                int  n = (int)NH;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        selection.hessian.cview(),
                        selection.hessian_offsets.cview(),
                        collected_dytopo_effect_hessian.cviewer(),
                        classified_hessians.viewer(),
                        classify_info.hessian_i_range(),
                        classify_info.hessian_j_range(),
                        n);
                }
            }

            classified_info.m_hessians = classified_hessians.view();
        }

        receiver->receive(classified_info);
    }
}

void GlobalDyTopoEffectManager::Impl::loose_resize_entries(
    cuda_tool::DeviceTripletMatrix<Float, 3>& m, SizeT size)
{
    if(size > m.triplet_capacity())
    {
        m.reserve_triplets_discard(size * reserve_ratio);
    }
    m.resize_triplets_discard(size);
}

void GlobalDyTopoEffectManager::Impl::loose_resize_entries(
    cuda_tool::DeviceDoubletVector<Float, 3>& v, SizeT size)
{
    if(size > v.doublet_capacity())
    {
        v.reserve_doublets_discard(size * reserve_ratio);
    }
    v.resize_doublets_discard(size);
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void GlobalDyTopoEffectManager::init()
{
    m_impl.init(world());
}

void GlobalDyTopoEffectManager::compute_dytopo_effect(ComputeDyTopoEffectInfo& info)
{
    m_impl.compute_dytopo_effect(info);
}

SizeT GlobalDyTopoEffectManager::assembly_blocks()
{
    SizeT blocks = 0;
    for(auto&& reporter : m_impl.dytopo_effect_reporters.view())
        blocks += reporter->report_assembly_blocks();
    return blocks;
}

void GlobalDyTopoEffectManager::compute_dytopo_effect()
{
    ComputeDyTopoEffectInfo info;
    m_impl.compute_dytopo_effect(info);
}

void GlobalDyTopoEffectManager::add_reporter(DyTopoEffectReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    auto flag = reporter->component_flags();
    UIPC_ASSERT(is_valid_flag(flag),
                "reporter component_flags() is not valid single flag, it's {}",
                enum_flags_name(flag));
    m_impl.dytopo_effect_reporters.register_sim_system(*reporter);

    // classify into contact / non-contact
    if(reporter->component_flags() == EnergyComponentFlags::Contact)
    {
        m_impl.contact_reporters.register_sim_system(*reporter);
    }
    else
    {
        m_impl.non_contact_reporters.register_sim_system(*reporter);
    }
}

void GlobalDyTopoEffectManager::add_receiver(DyTopoEffectReceiver* receiver)
{
    check_state(SimEngineState::BuildSystems, "add_receiver()");
    UIPC_ASSERT(receiver != nullptr, "receiver is nullptr");
    m_impl.dytopo_effect_receivers.register_sim_system(*receiver);
}
}  // namespace uipc::backend::cuda
