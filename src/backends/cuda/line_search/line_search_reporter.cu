/*
 * NVIDIA contributions: side-stream energy launches, the early launch of
 * positional energies and the prelaunch hook.
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

#include <line_search/line_search_reporter.h>
#include <typeinfo>

namespace uipc::backend::cuda
{
void LineSearchReporter::do_build(BuildInfo& info) {}

void LineSearchReporter::do_reduce_energy(LineSearcher::ComputeEnergyInfo& info)
{
}

bool LineSearchReporter::do_reads_active_set() const noexcept
{
    return true;
}

bool LineSearchReporter::do_prelaunch_energy(LineSearcher::ComputeEnergyInfo& info)
{
    return false;
}

void LineSearchReporter::do_build()
{
    auto& line_searcher = require<LineSearcher>();

    BuildInfo info;
    do_build(info);

    line_searcher.add_reporter(this);
}

void LineSearchReporter::record_start_point(LineSearcher::RecordInfo& info)
{
    do_record_start_point(info);
}

void LineSearchReporter::step_forward(LineSearcher::StepInfo& info)
{
    do_step_forward(info);
}

void LineSearchReporter::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

bool LineSearchReporter::prelaunch_energy(LineSearcher::ComputeEnergyInfo& info)
{
    return do_prelaunch_energy(info);
}

void LineSearchReporter::reduce_energy(LineSearcher::ComputeEnergyInfo& info)
{
    do_reduce_energy(info);
}

bool LineSearchReporter::reads_active_set() const noexcept
{
    return do_reads_active_set();
}

void LineSearchReporter::init()
{
    InitInfo info;
    do_init(info);
}
}  // namespace uipc::backend::cuda
