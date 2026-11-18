/*
 * NVIDIA contributions: the prelaunched energy extent and the report of
 * one-warp assembly blocks.
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

#include <dytopo_effect_system/dytopo_effect_reporter.h>

namespace uipc::backend::cuda
{
void DyTopoEffectReporter::do_build()
{
    auto& manager = require<GlobalDyTopoEffectManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void DyTopoEffectReporter::init()
{
    InitInfo info;
    do_init(info);
}

void DyTopoEffectReporter::do_init(InitInfo&) {}

void DyTopoEffectReporter::report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info)
{
    do_report_energy_extent(info);
}

bool DyTopoEffectReporter::do_report_prelaunch_energy_extent(GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo&)
{
    return false;
}

SizeT DyTopoEffectReporter::do_report_assembly_blocks()
{
    return 0;
}

SizeT DyTopoEffectReporter::report_assembly_blocks()
{
    return do_report_assembly_blocks();
}

bool DyTopoEffectReporter::report_prelaunch_energy_extent(GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo& info)
{
    return do_report_prelaunch_energy_extent(info);
}

void DyTopoEffectReporter::report_gradient_hessian_extent(GlobalDyTopoEffectManager::GradientHessianExtentInfo& info)
{
    do_report_gradient_hessian_extent(info);

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_count);
}

void DyTopoEffectReporter::assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info)
{
    do_assemble(info);

    m_impl.gradients = info.gradients();
    m_impl.hessians  = info.hessians();
}

void DyTopoEffectReporter::compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info)
{
    do_compute_energy(info);

    m_impl.energies = info.energies();
}
}  // namespace uipc::backend::cuda
