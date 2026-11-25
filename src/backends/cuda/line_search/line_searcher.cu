/*
 * NVIDIA contributions: concurrent energy producers, the cached energy graph,
 * the asynchronous initial energy, batched start-point copies and step
 * forward, read-back buffers and the prelaunched trial energies.
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

#include <line_search/line_searcher.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <line_search/line_search_reporter.h>
#include <animator/global_animator.h>
#include <sim_engine.h>
#include <uipc/common/timer.h>
#include <cuda_tool/cub.h>
#include <algorithm>
#include <utility>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LineSearcher);

LineSearcher::~LineSearcher()
{
    if(m_fixed_done)
        cudaEventDestroy(m_fixed_done);
    if(m_initial_energy_ready)
        cudaEventDestroy(m_initial_energy_ready);
}

void LineSearcher::do_build()
{
    // The animation substep ratio enters animator energies by value.
    m_global_animator = find<GlobalAnimator>();
}

void LineSearcher::init()
{
    auto scene = world().scene();

    auto report_enery_attr = scene.config().find<IndexT>("line_search/report_energy");
    m_report_energy = report_enery_attr->view()[0] != 0;

    auto max_iter_attr = scene.config().find<IndexT>("line_search/max_iter");
    m_max_iter         = max_iter_attr->view()[0];

    auto cubic_attr = scene.config().find<IndexT>("line_search/cubic_refinement");
    m_cubic_refinement = cubic_attr && cubic_attr->view()[0] != 0;
    // Only the standard IPC Newton loop refines backtracked steps; the
    // AL-IPC line search ignores the key.
    auto constitution_attr = scene.config().find<std::string>("contact/constitution");
    if(m_cubic_refinement && constitution_attr && constitution_attr->view()[0] != "ipc")
        logger::warn("line_search/cubic_refinement = 1 has no effect with contact/constitution = '{}': only the standard IPC line search refines backtracked steps",
                     constitution_attr->view()[0]);

    m_dt_attr = scene.config().find<Float>("dt");
    UIPC_ASSERT(m_dt_attr, "Scene config must have a 'dt' attribute.");

    const auto energy_count = m_reporters.view().size() + 1;
    m_device_energy_values.resize(energy_count);
    m_energy_values.resize(energy_count, 0);
    m_energy_set.resize(m_reporters.view().size(), 0);
    m_reporter_prelaunched.resize(m_reporters.view().size(), 0);

    // The evaluation is bound by its host launches; four streams keep the
    // long single-block energy kernels (ortho, joints, contact) apart for
    // few event calls.
    constexpr SizeT EnergyStreamCount = 4;
    m_energy_streams.create(EnergyStreamCount);
    m_fixed_streams.create(EnergyStreamCount);
    m_fixed_reduce_streams.create(EnergyStreamCount);
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_fixed_done, cudaEventDisableTiming));
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_initial_energy_ready, cudaEventDisableTiming));
    m_initial_energy_values.resize_discard(energy_count);

    auto reporter_view = m_reporters.view();

    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;

    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();

    m_has_fixed_energy =
        std::any_of(reporter_view.begin(),
                    reporter_view.end(),
                    [](LineSearchReporter* R) { return !R->reads_active_set(); });
}

void LineSearcher::record_start_point()
{
    cuda_tool::CopyBatch copies;
    record_start_point(copies);
    copies.launch();
}

void LineSearcher::record_start_point(cuda_tool::CopyBatch& copies)
{
    for(auto&& R : m_reporters.view())
    {
        RecordInfo info;
        info.m_copies = &copies;
        R->record_start_point(info);
    }
}

void LineSearcher::step_forward(Float alpha)
{
    for(auto&& R : m_reporters.view())
    {
        StepInfo info;
        info.alpha = alpha;
        R->step_forward(info);
    }
}

void LineSearcher::step_forward(StepBatch& steps)
{
    for(auto&& R : m_reporters.view())
    {
        StepInfo info;
        info.alpha   = steps.alpha();
        info.m_batch = &steps;
        R->step_forward(info);
    }
}

LineSearcher::FixedKey LineSearcher::fixed_key()
{
    FixedKey key;
    key.dt = m_dt_attr->view()[0];
    key.substep_ratio = m_global_animator ? m_global_animator->substep_ratio() : 1.0;
    key.extent_generation =
        m_global_animator ? m_global_animator->energy_extent_generation() : 0;
    key.allocation_epoch = cuda_tool::device_allocation_epoch();
    return key;
}

void LineSearcher::record_fixed_energy(cudaStream_t origin, bool capturing)
{
    auto reporter_view = m_reporters.view();

    // Producers on the fixed side streams, ordered after `origin`. Every
    // reporter reduces on a reduce stream of its own, which waits only for
    // the side streams its producers took (join_next, recorded right after
    // them), not for the other reporters' producers: each reporter's
    // reductions start as soon as its own producers finish.
    m_fixed_streams.fork(origin);
    m_fixed_reduce_streams.fork(origin);
    vector<std::pair<SizeT, cudaStream_t>> reduce_on;
    for(auto&& [i, R] : enumerate(reporter_view))
    {
        if(R->reads_active_set())
            continue;
        ComputeEnergyInfo info{
            this, cuda_tool::VarView<Float>{m_device_energy_values.data() + i}};
        info.m_side_streams = &m_fixed_streams;
        R->compute_energy(info);
        m_energy_set[i] = info.m_energy_set;
        cudaStream_t rs =
            m_fixed_reduce_streams[reduce_on.size() % m_fixed_reduce_streams.size()];
        m_fixed_streams.join_next(rs);
        reduce_on.emplace_back(i, rs);
    }

    // Without capture, the reductions also follow the work a producer that
    // ignores its stream enqueued on the legacy stream (a capture is
    // invalidated by such a launch, see FixedKey).
    if(!capturing)
        for(SizeT k = 0; k < std::min(reduce_on.size(), m_fixed_reduce_streams.size()); ++k)
            m_fixed_reduce_streams.fork_one(k, cuda_tool::default_stream());
    for(auto&& [i, rs] : reduce_on)
    {
        auto*             R = reporter_view[i];
        ComputeEnergyInfo info{
            this, cuda_tool::VarView<Float>{m_device_energy_values.data() + i}};
        info.m_stream = rs;
        R->reduce_energy(info);
        UIPC_ASSERT(m_energy_set[i] || info.m_energy_set,
                    "Energy[{}] not set by reporter, did you forget to call energy()?",
                    R->name());
        m_energy_set[i] = 1;
    }

    if(capturing)
    {
        m_fixed_reduce_streams.join(origin);
        m_fixed_streams.join(origin);
    }
    else
    {
        cudaStream_t done_stream = m_fixed_reduce_streams[0];
        m_fixed_reduce_streams.join(done_stream);
        m_fixed_streams.join(done_stream);
        CUDA_TOOL_CHECK(cudaEventRecord(m_fixed_done, done_stream));
    }
}

void LineSearcher::launch_fixed_energy()
{
    if(!m_has_fixed_energy)
        return;

    // The graph replays the launches of the last capture, which hold the
    // buffers and by-value arguments of that time: within one key they do
    // not change (see FixedKey), and the first evaluation with a new key
    // runs directly, so every buffer it sizes has its size when the next
    // one is captured. The key is taken again after that evaluation: what it
    // allocated is in the buffers the capture records.
    FixedKey key = fixed_key();
    if(!(key == m_fixed_key))
    {
        m_fixed_key   = key;
        m_fixed_state = FixedGraphState::Stale;
    }

    if(m_fixed_state == FixedGraphState::Warm && !m_fixed_graph.disabled())
    {
        auto result   = m_fixed_graph.recapture([this](cudaStream_t s)
                                              { record_fixed_energy(s, true); });
        m_fixed_state = result == cuda_tool::GraphCapture::Result::Ok ?
                            FixedGraphState::Ready :
                            FixedGraphState::Stale;
    }

    if(m_fixed_state == FixedGraphState::Ready)
    {
        cudaStream_t s = m_fixed_streams[0];
        m_fixed_streams.fork_one(0, cuda_tool::default_stream());
        CUDA_TOOL_CHECK(m_fixed_graph.launch(s));
        CUDA_TOOL_CHECK(cudaEventRecord(m_fixed_done, s));
        return;
    }

    record_fixed_energy(cuda_tool::default_stream(), false);
    m_fixed_key = fixed_key();
    if(!m_fixed_graph.disabled())
        m_fixed_state = FixedGraphState::Warm;
}

void LineSearcher::enqueue_energy(bool is_initial, Float* total_out)
{
    auto reporter_view = m_reporters.view();

    // The fixed part runs on its own streams (launched by launch_energy()
    // or here); the producers of the other reporters run concurrently on
    // the side streams, forked after the trial's filter_active, and those
    // reporters reduce their output on the default stream after one join.
    // The default stream then waits for the fixed part before the final
    // sum, so every sum keeps its inputs and order.
    bool launched = std::exchange(m_energy_launched, false);
    UIPC_ASSERT(!(launched && is_initial), "A launched energy is a trial energy");
    if(!launched)
        launch_fixed_energy();

    // After a prelaunch, the rest of the evaluation runs on the legacy
    // stream behind the prelaunched producers.
    bool prelaunched = std::exchange(m_energy_prelaunched, false);
    UIPC_ASSERT(!(prelaunched && is_initial), "A prelaunched energy is a trial energy");
    if(prelaunched)
        m_energy_streams.join_next();
    else
        m_energy_streams.fork();
    for(auto&& [i, R] : enumerate(reporter_view))
    {
        if(!R->reads_active_set())
            continue;
        ComputeEnergyInfo info{
            this, cuda_tool::VarView<Float>{m_device_energy_values.data() + i}};
        info.m_is_initial   = is_initial;
        info.m_side_streams = prelaunched ? nullptr : &m_energy_streams;
        info.m_prelaunched  = prelaunched && m_reporter_prelaunched[i];
        R->compute_energy(info);
        m_energy_set[i] = info.m_energy_set;
    }
    m_energy_streams.join_next();
    for(auto&& [i, R] : enumerate(reporter_view))
    {
        if(!R->reads_active_set())
            continue;
        ComputeEnergyInfo info{
            this, cuda_tool::VarView<Float>{m_device_energy_values.data() + i}};
        info.m_is_initial = is_initial;
        R->reduce_energy(info);
        UIPC_ASSERT(m_energy_set[i] || info.m_energy_set,
                    "Energy[{}] not set by reporter, did you forget to call energy()?",
                    R->name());
    }
    CUDA_TOOL_CHECK(cudaStreamWaitEvent(cuda_tool::default_stream(), m_fixed_done, 0));

    const auto reporter_count = reporter_view.size();
    auto*      total_energy_device =
        total_out ? total_out : m_device_energy_values.data() + reporter_count;
    if(reporter_count > 0)
    {
        cuda_tool::DeviceReduce().Sum(m_device_energy_values.data(), total_energy_device, reporter_count);
    }
    else
    {
        // total_out may be mapped host memory (no host-to-device copy there)
        cuda_tool::store_value(total_energy_device, Float{0}, cuda_tool::default_stream());
    }
}

Float LineSearcher::compute_energy(bool is_initial)
{
    Timer timer{"Compute Energy"};

    // The sum stores the total into a read-back buffer (on the mapped path
    // no copy follows it). A finite total implies finite reporter energies
    // (a NaN or an infinity among the addends makes the sum NaN or
    // infinite), so the reporters' values are copied and checked (and
    // reported) only when the total is not finite or reporting is on: the
    // same checks fail with the same messages.
    m_total_energy.resize_discard(1);
    enqueue_energy(is_initial, m_total_energy.device_data());
    m_total_energy.enqueue_copy(cuda_tool::default_stream());
    CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
    Float total = m_total_energy[0];
    if(!std::isfinite(total) || m_report_energy)
    {
        m_device_energy_values.copy_to(m_energy_values.data());
        m_energy_values.back() = total;
        return read_energy(m_energy_values);
    }
    return total;
}

void LineSearcher::launch_initial_energy()
{
    Timer timer{"Compute Energy"};

    // A line search that threw before reading its E_0 left it pending; the
    // copy is complete before this one in stream order.
    if(std::exchange(m_initial_energy_pending, false))
        CUDA_TOOL_CHECK(cudaEventSynchronize(m_initial_energy_ready));
    enqueue_energy(true);
    m_initial_energy_values.copy_from(m_device_energy_values.cview());
    CUDA_TOOL_CHECK(cudaEventRecord(m_initial_energy_ready, cuda_tool::default_stream()));
    m_initial_energy_pending = true;
}

Float LineSearcher::initial_energy()
{
    UIPC_ASSERT(m_initial_energy_pending, "No initial energy is launched");
    CUDA_TOOL_CHECK(cudaEventSynchronize(m_initial_energy_ready));
    m_initial_energy_pending = false;
    return read_energy(span<const Float>{m_initial_energy_values.data(),
                                         m_initial_energy_values.size()});
}

Float LineSearcher::read_energy(span<const Float> values)
{
    auto       reporter_view  = m_reporters.view();
    const auto reporter_count = reporter_view.size();

    auto reporter_energies = values.first(reporter_count);
    for(auto&& [energy, reporter] : zip(reporter_energies, reporter_view))
    {
        UIPC_ASSERT(!std::isnan(energy) && std::isfinite(energy),
                    "Energy [{}] is {}",
                    reporter->name(),
                    energy);
    }

    Float total_energy = values.back();
    UIPC_ASSERT(!std::isnan(total_energy) && std::isfinite(total_energy),
                "Total line-search energy is {}",
                total_energy);

    if(m_report_energy)
    {
        m_report_stream << R"(
-------------------------------------------------------------------------------
*                             Compute Energy                                  *
-------------------------------------------------------------------------------
)";
        m_report_stream << "Total:" << total_energy << "\n";
        for(auto&& [R, value] : zip(reporter_view, reporter_energies))
        {
            m_report_stream << "  > " << R->name() << "=" << value << "\n";
        }
        m_report_stream << "-------------------------------------------------------------------------------";
        logger::info(m_report_stream.str());
        m_report_stream.str("");
    }

    return total_energy;
}

void LineSearcher::launch_energy()
{
    Timer timer{"Launch Energy"};

    UIPC_ASSERT(!m_energy_launched, "An energy is launched already");
    launch_fixed_energy();
    m_energy_launched = true;
}

void LineSearcher::discard_energy()
{
    // The next step overwrites what the launched producers read.
    if(std::exchange(m_energy_launched, false))
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(cuda_tool::default_stream(), m_fixed_done, 0));
    if(std::exchange(m_energy_prelaunched, false))
        m_energy_streams.join_next();
}

void LineSearcher::prelaunch_energy(cudaEvent_t filtered)
{
    // A prelaunch left by a trial that threw is discarded.
    if(std::exchange(m_energy_prelaunched, false))
        m_energy_streams.join_next();

    for(auto&& [i, R] : enumerate(m_reporters.view()))
    {
        m_reporter_prelaunched[i] = 0;
        if(!R->reads_active_set())
            continue;
        ComputeEnergyInfo info{
            this, cuda_tool::VarView<Float>{m_device_energy_values.data() + i}};
        info.m_side_streams       = &m_energy_streams;
        info.m_filtered           = filtered;
        m_reporter_prelaunched[i] = R->prelaunch_energy(info);
        m_energy_prelaunched |= m_reporter_prelaunched[i] != 0;
    }
}

void LineSearcher::add_reporter(LineSearchReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is nullptr");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_reporters.register_sim_system(*reporter);
}

LineSearcher::ComputeEnergyInfo::ComputeEnergyInfo(LineSearcher* impl,
                                                   cuda_tool::VarView<Float> energy) noexcept
    : m_impl(impl)
    , m_energy(energy)
{
}

Float LineSearcher::ComputeEnergyInfo::dt() noexcept
{
    return m_impl->m_dt_attr->view()[0];
}

cuda_tool::VarView<Float> LineSearcher::ComputeEnergyInfo::energy() noexcept
{
    m_energy_set = true;
    return m_energy;
}

bool LineSearcher::ComputeEnergyInfo::is_initial() noexcept
{
    return m_is_initial;
}

cuda_tool::SideStreams* LineSearcher::ComputeEnergyInfo::side_streams() noexcept
{
    return m_side_streams;
}

cudaStream_t LineSearcher::ComputeEnergyInfo::stream() noexcept
{
    return m_stream;
}

cudaEvent_t LineSearcher::ComputeEnergyInfo::filtered() noexcept
{
    return m_filtered;
}

bool LineSearcher::ComputeEnergyInfo::prelaunched() noexcept
{
    return m_prelaunched;
}

SizeT LineSearcher::max_iter() const noexcept
{
    return m_max_iter;
}

bool LineSearcher::cubic_refinement() const noexcept
{
    return m_cubic_refinement;
}
}  // namespace uipc::backend::cuda
