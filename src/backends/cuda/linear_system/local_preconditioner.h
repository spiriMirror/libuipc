/*
 * NVIDIA contributions: optional fused-PCG preconditioner interfaces and the
 * default unsupported-path fallback, and the block-diagonal inverse and
 * assembly-preparation interfaces.
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

#pragma once
#include <sim_system.h>
#include <linear_system/global_linear_system.h>

namespace uipc::backend::cuda
{
class DiagLinearSubsystem;

class LocalPreconditioner : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class InitInfo
    {
      public:
    };

    class BuildInfo
    {
      public:
        void connect(DiagLinearSubsystem* system);

      private:
        friend class LocalPreconditioner;
        DiagLinearSubsystem* m_subsystem = nullptr;
    };

    // Before the global matrix is converted, in a Newton iteration whose
    // preconditioners are assembled (do_assemble() follows).
    class PrepareAssemblyInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo& info)   = 0;
    // Optional: work of the coming do_assemble() that does not read the
    // global matrix (it is not converted yet), enqueued so that it runs
    // beside the conversion, e.g. clearing buffers on a side stream.
    virtual void do_prepare_assembly(PrepareAssemblyInfo& info) {}
    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) = 0;
    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) = 0;
    virtual bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info)
    {
        return false;
    }
    // Report the block-diagonal inverse whose fused step equals
    // do_fused_pcg_update_apply_dot; only meaningful when that returns true.
    virtual bool do_fused_pcg_diag_inverse(GlobalLinearSystem::FusedPcgDiagInverseInfo& info)
    {
        return false;
    }

  private:
    friend class GlobalLinearSystem;

    virtual void do_build() final override;
    virtual void init();

    void prepare_assembly(PrepareAssemblyInfo& info);
    void assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info);
    void apply(GlobalLinearSystem::ApplyPreconditionerInfo& info);
    bool fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info);
    bool fused_pcg_diag_inverse(GlobalLinearSystem::FusedPcgDiagInverseInfo& info);
    DiagLinearSubsystem* m_subsystem = nullptr;
};
}  // namespace uipc::backend::cuda
