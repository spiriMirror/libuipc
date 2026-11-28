/*
 * NVIDIA contributions: tolerance checks launched on the line-search prologue
 * stream.
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

#include <newton_tolerance/newton_tolerance_manager.h>
#include <newton_tolerance/newton_tolerance_checker.h>
#include <utility>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(NewtonToleranceManager);

void NewtonToleranceManager::do_build() {}

void NewtonToleranceManager::Impl::init()
{
    auto checkers = tolerance_checkers.view();

    for(auto& checker : checkers)
    {
        checker->init();  // Initialize each registered checker
    }

    report_buffer.reserve(1024);  // Reserve space for report buffer
}

void NewtonToleranceManager::add_checker(NewtonToleranceChecker* checker)
{
    check_state(SimEngineState::BuildSystems, "add_checker()");
    UIPC_ASSERT(checker, "Cannot add null NewtonToleranceChecker");
    m_impl.tolerance_checkers.register_sim_system(*checker);
}

void NewtonToleranceManager::init()
{
    m_impl.init();
}

void NewtonToleranceManager::pre_newton(SizeT frame)
{
    m_impl.launched = false;
    auto checkers   = m_impl.tolerance_checkers.view();

    for(auto& checker : checkers)
    {
        PreNewtonInfo pre_info;
        pre_info.m_frame = frame;
        checker->pre_newton(pre_info);
    }
}

void NewtonToleranceManager::launch_check(cudaStream_t stream)
{
    for(auto& checker : m_impl.tolerance_checkers.view())
    {
        LaunchCheckInfo launch_info;
        launch_info.m_stream = stream;
        checker->launch_check(launch_info);
    }
    m_impl.launch_stream = stream;
    m_impl.launched      = true;
}

void NewtonToleranceManager::check(ResultInfo& info)
{
    bool launched = std::exchange(m_impl.launched, false);
    if(launched)
        CUDA_TOOL_CHECK(cudaStreamSynchronize(m_impl.launch_stream));

    auto checkers      = m_impl.tolerance_checkers.view();
    bool all_converged = true;

    m_impl.report_buffer.clear();  // Clear the report buffer for this frame
    for(auto& checker : checkers)
    {
        CheckResultInfo this_info;
        // Input:
        this_info.m_newton_iter = info.m_newton_iter;
        this_info.m_launched    = launched;

        checker->check(this_info);
        std::string report = checker->report();

        fmt::format_to(std::back_inserter(m_impl.report_buffer),
                       "\n    [{}] <{}> {}",
                       this_info.m_converged ? "*" : "x",
                       checker->name(),
                       report);

        // Output:
        if(!this_info.m_converged)
        {
            all_converged = false;
            break;
        }
    }

    logger::info(
        R"(>> Newton Tolerance Checker
============================= [{}] Frame {} Newton Iteration {} ============================{}
--------------------------------------------------------------------------------------------)",
        all_converged ? "*" : "x",
        info.m_frame,
        info.m_newton_iter,
        m_impl.report_buffer);

    info.m_converged = all_converged;
}
}  // namespace uipc::backend::cuda