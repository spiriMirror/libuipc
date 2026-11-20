/*
 * NVIDIA contributions: prelaunch hooks for the contact-independent assembly
 * and hooks for late Hessian blocks.
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

#include <linear_system/diag_linear_subsystem.h>

namespace uipc::backend::cuda
{
U64 DiagLinearSubsystem::uid() const noexcept
{
    return get_uid();
}

IndexT DiagLinearSubsystem::dof_offset() const noexcept
{
    auto& offset_counts = m_global_linear_system->m_impl.diag_dof_offsets_counts;
    auto offsets = offset_counts.offsets();
    UIPC_ASSERT(m_index < offsets.size(),
                "Invalid subsystem index, Linear Subsystem Count = {}, Your Subsystem ({}) Index = {}",
                offsets.size(),
                name(),
                m_index);
    return offsets[m_index];
}

IndexT DiagLinearSubsystem::dof_count() const noexcept
{
    auto& offset_counts = m_global_linear_system->m_impl.diag_dof_offsets_counts;
    auto counts = offset_counts.counts();
    UIPC_ASSERT(m_index < counts.size(),
                "Invalid subsystem index, Linear Subsystem Count = {}, Your Subsystem ({}) Index = {}",
                counts.size(),
                name(),
                m_index);
    return counts[m_index];
}

void DiagLinearSubsystem::do_build(BuildInfo& info) {}

void DiagLinearSubsystem::do_build()
{
    m_global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_global_linear_system->add_subsystem(this);
}

void DiagLinearSubsystem::init()
{
    InitInfo info;
    do_init(info);
}

void DiagLinearSubsystem::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    do_report_init_extent(info);
}

void DiagLinearSubsystem::receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    do_receive_init_dof_info(info);
}

void DiagLinearSubsystem::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    do_report_extent(info);
}

void DiagLinearSubsystem::do_prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info)
{
}

void DiagLinearSubsystem::prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info)
{
    do_prepare_prelaunch(info);
}

void DiagLinearSubsystem::do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
}

void DiagLinearSubsystem::prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
    do_prelaunch_assembly(info);
}

void DiagLinearSubsystem::assemble(GlobalLinearSystem::DiagInfo& info)
{
    UIPC_ASSERT(info.gradient_only()
                    || info.component_flags() == GlobalLinearSystem::ComponentFlags::All,
                "Limitation: When info.gradient_only()==false, info.component_flags() must be GlobalLinearSystem::ComponentFlags::All (got {})",
                enum_flags_name(info.component_flags()));

    do_assemble(info);
}

void DiagLinearSubsystem::do_report_late_extent(GlobalLinearSystem::LateExtentInfo& info)
{
}

void DiagLinearSubsystem::report_late_extent(GlobalLinearSystem::LateExtentInfo& info)
{
    do_report_late_extent(info);
}

void DiagLinearSubsystem::do_assemble_late(GlobalLinearSystem::DiagInfo& info)
{
    UIPC_ASSERT(info.hessians().triplet_count() == 0,
                "{} reported late Hessian blocks but does not assemble them",
                name());
}

void DiagLinearSubsystem::assemble_late(GlobalLinearSystem::DiagInfo& info)
{
    do_assemble_late(info);
}

void DiagLinearSubsystem::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    do_accuracy_check(info);
}
void DiagLinearSubsystem::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    do_retrieve_solution(info);
}

Float DiagLinearSubsystem::diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return do_diag_norm(info);
}

Float DiagLinearSubsystem::mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return do_mass_norm(info);
}
}  // namespace uipc::backend::cuda
