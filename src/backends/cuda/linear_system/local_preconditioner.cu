/*
 * NVIDIA contributions: fused-PCG preconditioner dispatch and the
 * unsupported-path fallback, and the block-diagonal inverse and
 * assembly-preparation hooks.
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

#include <linear_system/local_preconditioner.h>

namespace uipc::backend::cuda
{
void LocalPreconditioner::do_build()
{
    auto& global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_subsystem = info.m_subsystem;
    UIPC_ASSERT(m_subsystem != nullptr, "The depend_subsystem should not be nullptr.");

    global_linear_system.add_preconditioner(this);
}

void LocalPreconditioner::init()
{
    InitInfo info;
    do_init(info);
}

void LocalPreconditioner::prepare_assembly(PrepareAssemblyInfo& info)
{
    do_prepare_assembly(info);
}

void LocalPreconditioner::assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    do_assemble(info);
}

void LocalPreconditioner::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    do_apply(info);
}

bool LocalPreconditioner::fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info)
{
    return do_fused_pcg_update_apply_dot(info);
}

bool LocalPreconditioner::fused_pcg_diag_inverse(GlobalLinearSystem::FusedPcgDiagInverseInfo& info)
{
    return do_fused_pcg_diag_inverse(info);
}

void LocalPreconditioner::BuildInfo::connect(DiagLinearSubsystem* system)
{
    m_subsystem = system;
}
}  // namespace uipc::backend::cuda
