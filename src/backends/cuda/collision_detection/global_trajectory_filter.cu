/*
 * NVIDIA contributions: line-search trajectory-filter lifecycle and
 * active-contact count integration; frame-start active-set reuse with its
 * validation mode, trajectory-candidate reuse within a detection skin, TOI and
 * two-phase active-set filtering on side streams, batched, copy-free
 * read-backs, the count read-back behind prelaunched work and the detection
 * hook before the candidate-count read-back.
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

#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <cuda_tool/cub.h>
#include <sim_engine.h>
#include <algorithm>
#include <cstdlib>
#include <string_view>
#include <utility>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalTrajectoryFilter>
{
  public:
    static U<cuda::GlobalTrajectoryFilter> create(cuda::SimEngine& engine)
    {
        auto contact_enable = engine.world().scene().config().find<IndexT>("contact/enable");
        if(contact_enable->view()[0])
            return make_unique<cuda::GlobalTrajectoryFilter>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend


namespace uipc::backend::cuda
{
namespace
{
    // Pair-distance skin of the reusable (Newton-loop) trajectory detections,
    // relative to the smallest per-vertex d_hat. reuse_candidates needs
    // skin <= d_hat / 2. A larger skin lets more iterations reuse but grows
    // every candidate list; 0.25 balances the two.
    constexpr Float CandidateSkinRatio = 0.25;
    static_assert(CandidateSkinRatio > 0.0 && CandidateSkinRatio <= 0.5);

    // Compares by value: a NaN counts as moved, signed zeros compare equal.
    // The half-plane reporter rewrites its fixed plane points (read by no
    // filter) every frame, restoring a -0.0 that step_forward made +0.0.
    __global__ void GlobalTrajectoryFilter_flag_moved_vertices_kernel(
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Vector3> recorded,
        cuda_tool::Dense<IndexT>        moved,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const Vector3& x = positions(i);
        const Vector3& r = recorded(i);
        for(int k = 0; k < 3; ++k)
        {
            if(!(x[k] == r[k]))
            {
                *moved.data() = 1;
                return;
            }
        }
    }

    // Host copy of an active-set buffer in lexicographic order, so that two
    // copies compare equal exactly when they hold the same multiset.
    template <typename T>
    vector<T> sorted_host_copy(cuda_tool::CBufferView<T> pairs)
    {
        vector<T> host(pairs.size());
        pairs.copy_to(host.data());
        std::ranges::sort(host,
                          [](const T& a, const T& b)
                          {
                              return std::lexicographical_compare(a.data(),
                                                                  a.data() + a.size(),
                                                                  b.data(),
                                                                  b.data() + b.size());
                          });
        return host;
    }
}  // namespace

REGISTER_SIM_SYSTEM(GlobalTrajectoryFilter);

GlobalTrajectoryFilter::~GlobalTrajectoryFilter()
{
    if(m_impl.filtered)
        cudaEventDestroy(m_impl.filtered);
    if(m_impl.side_filtered)
        cudaEventDestroy(m_impl.side_filtered);
}

void GlobalTrajectoryFilter::do_build()
{
    auto& config               = world().scene().config();
    auto  friction_enable_attr = config.find<IndexT>("contact/friction/enable");

    m_impl.friction_enabled = friction_enable_attr->view()[0];

    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();

    // Validation only: every reused frame-start active set is compared with
    // a full DCD pass (see check_reused_active_set).
    auto check                    = std::getenv("UIPC_CHECK_ACTIVE_SET_REUSE");
    m_impl.check_active_set_reuse = check && std::string_view{check} == "1";

    on_init_scene([&] { m_impl.init(); });
}

void GlobalTrajectoryFilter::add_filter(TrajectoryFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input TrajectoryFilter is nullptr.");
    m_impl.filters.register_sim_system(*filter);
}

void GlobalTrajectoryFilter::Impl::init()
{
    auto filter_view = filters.view();
    tois.resize_discard(filter_view.size());
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&filtered, cudaEventDisableTiming));
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&side_filtered, cudaEventDisableTiming));

    detect_skin_supported =
        std::ranges::all_of(filter_view,
                            [](TrajectoryFilter* f)
                            { return f->supports_detect_skin(); });
}

Float GlobalTrajectoryFilter::Impl::candidate_skin()
{
    if(!detect_skin_supported)
        return 0.0;
    // Per-vertex d_hats are uploaded once at init.
    if(min_d_hat < 0.0)
    {
        auto d_hats = global_vertex_manager->d_hats();
        if(d_hats.size() == 0)
            return 0.0;
        cuda_tool::DeviceReduce().Min(d_hats.data(),
                                      min_d_hat_reduced.data(),
                                      static_cast<int>(d_hats.size()));
        min_d_hat = min_d_hat_reduced;
    }
    return CandidateSkinRatio * min_d_hat;
}

void GlobalTrajectoryFilter::detect(Float alpha,
                                    bool  reuse_bvh_topology,
                                    bool  reusable,
                                    const std::function<void()>& before_read_back)
{
    Float skin = reusable && alpha > 0.0 ? m_impl.candidate_skin() : 0.0;
    for(auto filter : m_impl.filters.view())
    {
        DetectInfo info;
        info.m_alpha              = alpha;
        info.m_reuse_bvh_topology = reuse_bvh_topology;
        info.m_skin               = skin;
        info.m_before_read_back   = &before_read_back;
        filter->detect(info);
    }
    m_impl.held_skin = skin;
}

// A reusable detection over x -> x + d keeps every pair that its trajectory
// tests cannot place farther than d_hat + thickness + skin apart during that
// step. A later step x' -> x' + d' over which every vertex stays within
// axis_motion per axis of its detected swept segment moves each pair by at
// most 2 sqrt(3) axis_motion <= skin relative to its detected trajectory, so
// a pair the held candidates lack stays farther than d_hat + thickness apart
// over the later step: no line-search trial activates it, and ACCD would
// report no hit for it, because its first advancement step
// (1 - eta) (d0 - thickness) / maxDispMag > 0.9 d_hat / (3 sqrt(3)
// axis_motion) exceeds the initial toc of 1.1 while skin <= d_hat / 2. A
// held pair that a fresh detection would drop is inert for the same reasons.
// Hence the held and the fresh candidates give the same minimum TOI and the
// same active-set multisets; only the pair order may differ.
bool GlobalTrajectoryFilter::reuse_candidates(Float axis_motion) const
{
    // Tiny margin for the FP64 rounding of positions and drift sums.
    constexpr Float max_relative_motion = 2.0 * 1.7320508075688772 * (1.0 + 1e-6);
    return m_impl.held_skin > 0.0 && axis_motion * max_relative_motion <= m_impl.held_skin;
}

void GlobalTrajectoryFilter::filter_active(const std::function<void(cudaEvent_t)>& before_sync)
{
    if(m_impl.global_contact_manager->cfl_enabled())
    {
        auto is_acitive =
            m_impl.global_contact_manager->m_impl.vert_is_active_contact.view();
        is_acitive.fill(0);  // clear the active flag
    }

    // Filters with a two-phase filter_active enqueue their work and count
    // read-backs first; one synchronization then serves all of them.
    auto filter_view = m_impl.filters.view();
    m_impl.filter_active_launched.assign(filter_view.size(), 0);
    bool launched = false;
    // The filters on the legacy stream are launched first, then
    // before_sync's work behind them (they lead the trial's critical
    // chain); the ones that run beside (runs_beside(): the half-plane
    // filter) take a side stream ordered after the legacy stream's work
    // before the filters (the step), joined back after before_sync's work.
    bool beside =
        std::any_of(filter_view.begin(),
                    filter_view.end(),
                    [](TrajectoryFilter* f) { return f->runs_beside(); });
    if(beside)
        m_impl.filter_active_side.mark(cuda_tool::default_stream());
    for(auto&& [i, filter] : enumerate(filter_view))
    {
        if(filter->runs_beside())
            continue;
        FilterActiveInfo info(&m_impl);
        m_impl.filter_active_launched[i] = filter->launch_filter_active(info);
        launched |= m_impl.filter_active_launched[i] != 0;
    }
    // (every filter beside launches, see below)
    const bool sync_events = (launched || beside) && before_sync;
    if(sync_events)
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_impl.filtered, nullptr));
        before_sync(m_impl.filtered);
    }
    if(beside)
    {
        cudaStream_t side = m_impl.filter_active_side.fork_marked();
        for(auto&& [i, filter] : enumerate(filter_view))
        {
            if(!filter->runs_beside())
                continue;
            FilterActiveInfo info(&m_impl);
            info.m_stream = side;
            m_impl.filter_active_launched[i] = filter->launch_filter_active(info);
            UIPC_ASSERT(m_impl.filter_active_launched[i],
                        "{} runs beside but has no two-phase filter_active",
                        filter->name());
            launched = true;
        }
        CUDA_TOOL_CHECK(cudaEventRecord(m_impl.side_filtered, side));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(cuda_tool::default_stream(), m_impl.side_filtered, 0));
    }

    if(sync_events)
    {
        CUDA_TOOL_CHECK(cudaEventSynchronize(m_impl.filtered));
        if(beside)
            CUDA_TOOL_CHECK(cudaEventSynchronize(m_impl.side_filtered));
    }
    else if(launched)
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));

    for(auto&& [i, filter] : enumerate(filter_view))
    {
        FilterActiveInfo info(&m_impl);
        if(m_impl.filter_active_launched[i])
            filter->finish_filter_active(info);
        else
            filter->filter_active(info);
    }
}

Float GlobalTrajectoryFilter::Impl::filter_toi(Float alpha)
{
    launch_filter_toi(alpha, global_vertex_manager->positions(), cuda_tool::default_stream());
    return finish_filter_toi();
}

void GlobalTrajectoryFilter::Impl::launch_filter_toi(Float alpha,
                                                     cuda_tool::CBufferView<Vector3> positions,
                                                     cudaStream_t stream)
{
    auto filter_view = filters.view();
    // As in filter_active(), the filters that run beside take a side stream
    // ordered after `stream`'s work so far, joined back before the TOIs'
    // read-back.
    bool beside =
        std::any_of(filter_view.begin(),
                    filter_view.end(),
                    [](TrajectoryFilter* f) { return f->runs_beside(); });
    if(beside)
        filter_toi_side.mark(stream);
    for(int pass = 0; pass < (beside ? 2 : 1); ++pass)
    {
        cudaStream_t s = pass == 0 ? stream : filter_toi_side.fork_marked();
        for(auto&& [i, filter] : enumerate(filter_view))
        {
            if(filter->runs_beside() != (pass == 1))
                continue;
            FilterTOIInfo info;
            info.m_toi   = cuda_tool::VarView<Float>{tois.device_data() + i};
            info.m_alpha = alpha;
            info.m_positions = positions;
            info.m_stream    = s;
            filter->filter_toi(info);
        }
    }
    if(beside)
        filter_toi_side.join(stream);
    tois.enqueue_copy(stream);
    toi_stream = stream;
}

Float GlobalTrajectoryFilter::Impl::finish_filter_toi()
{
    CUDA_TOOL_CHECK(cudaStreamSynchronize(toi_stream));
    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto filter_view = filters.view();
        for(SizeT i = 0; i < tois.size(); ++i)
        {
            UIPC_ASSERT(tois[i] > 0.0f,
                        "Invalid toi[{}] value: {}",
                        filter_view[i]->name(),
                        tois[i]);
        }
    }

    auto min_toi = *std::min_element(tois.data(), tois.data() + tois.size());

    return min_toi < 1.0 ? min_toi : 1.0;
}

Float GlobalTrajectoryFilter::filter_toi(Float alpha)
{
    return m_impl.filter_toi(alpha);
}

void GlobalTrajectoryFilter::launch_filter_toi(Float alpha, cudaStream_t stream)
{
    // The line search starts from the recorded start point: its positions
    // equal the current positions until the first trial steps forward.
    m_impl.launch_filter_toi(alpha, m_impl.global_vertex_manager->safe_positions(), stream);
}

Float GlobalTrajectoryFilter::finish_filter_toi()
{
    return m_impl.finish_filter_toi();
}

void GlobalTrajectoryFilter::record_friction_candidates()
{
    // Check if friction candidates should be discarded before recording new ones
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    if(m_impl.should_discard_friction_candidates)
    {
        clear_friction_candidates();
        m_impl.should_discard_friction_candidates = false;
        // No need to record friction candidates if they are discarded
        // Just early return
        return;
    }

    for(auto filter : m_impl.filters.view())
    {
        RecordFrictionCandidatesInfo info;
        filter->record_friction_candidates(info);
    }
}

void GlobalTrajectoryFilter::label_active_vertices()
{
    for(auto filter : m_impl.filters.view())
    {
        LabelActiveVerticesInfo info(&m_impl);
        filter->label_active_vertices(info);
    }
}

void GlobalTrajectoryFilter::clear_friction_candidates()
{
    for(auto filter : m_impl.filters.view())
    {
        filter->clear_friction_candidates();
    }
}

void GlobalTrajectoryFilter::require_discard_friction()
{
    m_impl.should_discard_friction_candidates = true;
    // The vertex attributes were rewritten: the next frame runs the DCD pass.
    m_impl.active_set_recorded = false;
}

void GlobalTrajectoryFilter::record_active_set()
{
    auto positions = m_impl.global_vertex_manager->positions();
    m_impl.active_set_positions.resize_discard(positions.size());
    if(positions.size() > 0)
        cuda_tool::BufferLaunch().copy<Vector3>(m_impl.active_set_positions.view(), positions);
    m_impl.active_set_recorded = true;
}

bool GlobalTrajectoryFilter::reuse_active_set()
{
    // A recorded active set serves only the frame that follows it; a frame
    // that throws before record_active_set leaves none behind.
    if(!std::exchange(m_impl.active_set_recorded, false))
        return false;

    auto positions = m_impl.global_vertex_manager->positions();
    if(positions.size() != m_impl.active_set_positions.size())
        return false;

    int n = static_cast<int>(positions.size());
    if(n > 0)
    {
        m_impl.moved_vertices = 0;
        auto k = GlobalTrajectoryFilter_flag_moved_vertices_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            positions,
            m_impl.active_set_positions.cview(),
            m_impl.moved_vertices.viewer(),
            n);
        if(static_cast<IndexT>(m_impl.moved_vertices) != 0)
            return false;
    }

    if(m_impl.check_active_set_reuse)
        check_reused_active_set();
    return true;
}

void GlobalTrajectoryFilter::check_reused_active_set()
{
    auto simplex    = find<SimplexTrajectoryFilter>();
    auto half_plane = find<VertexHalfPlaneTrajectoryFilter>();

    struct ActiveSet
    {
        vector<Vector4i> PTs;
        vector<Vector4i> EEs;
        vector<Vector3i> PEs;
        vector<Vector2i> PPs;
        vector<Vector2i> PHs;
    };
    auto download = [&]
    {
        ActiveSet set;
        if(simplex)
        {
            set.PTs = sorted_host_copy(simplex->PTs());
            set.EEs = sorted_host_copy(simplex->EEs());
            set.PEs = sorted_host_copy(simplex->PEs());
            set.PPs = sorted_host_copy(simplex->PPs());
        }
        if(half_plane)
            set.PHs = sorted_host_copy(half_plane->PHs());
        return set;
    };

    ActiveSet reused = download();
    detect(0.0, false);
    filter_active();
    ActiveSet dcd = download();

    if(reused.PTs != dcd.PTs || reused.EEs != dcd.EEs || reused.PEs != dcd.PEs
       || reused.PPs != dcd.PPs || reused.PHs != dcd.PHs)
    {
        throw SimEngineException(fmt::format("Active-set reuse check failed at frame {}: reused/DCD PT {}/{}, EE {}/{}, PE {}/{}, PP {}/{}, PH {}/{}",
                                             engine().frame(),
                                             reused.PTs.size(),
                                             dcd.PTs.size(),
                                             reused.EEs.size(),
                                             dcd.EEs.size(),
                                             reused.PEs.size(),
                                             dcd.PEs.size(),
                                             reused.PPs.size(),
                                             dcd.PPs.size(),
                                             reused.PHs.size(),
                                             dcd.PHs.size()));
    }
}

cuda_tool::BufferView<IndexT> GlobalTrajectoryFilter::LabelActiveVerticesInfo::vert_is_active() const noexcept
{
    return m_impl->global_contact_manager->m_impl.vert_is_active_contact.view();
}

void GlobalTrajectoryFilter::do_apply_recover(RecoverInfo& info)
{
    // Friction candidates are already recovered, no need to discard them.
    m_impl.should_discard_friction_candidates = false;
    // The recovered positions need a DCD pass.
    m_impl.active_set_recorded = false;
    m_impl.held_skin           = 0.0;
}
}  // namespace uipc::backend::cuda
