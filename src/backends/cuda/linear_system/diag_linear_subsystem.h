/*
 * NVIDIA contributions: prelaunch interfaces for the contact-independent
 * assembly and interfaces for late Hessian blocks.
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
/**
 * @brief A diag linear subsystem represents a submatrix of the global hessian and a subvector of the global gradient.
 */
class DiagLinearSubsystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

    U64 uid() const noexcept;

    IndexT dof_offset() const noexcept;
    IndexT dof_count() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info);
    virtual void do_init(InitInfo& info) = 0;

    virtual void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) = 0;
    virtual void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info) = 0;

    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) = 0;
    // Optionally launch, on info.side_streams(), producers of the next
    // assembly that read no DyTopo (contact) results; do_assemble() then sees
    // DiagInfo::prelaunched() and must not launch them again.
    // do_prepare_prelaunch() runs first, before the DyTopo work is enqueued:
    // it sizes the producers' buffers and forks the side streams, so that the
    // launches themselves may be issued after the DyTopo assembly kernels.
    virtual void do_prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info);
    virtual void do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info);
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info)           = 0;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) = 0;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) = 0;
    // Late blocks (GlobalLinearSystem::DiagExtentInfo::late_extent()): their
    // count, which may wait for the device, once every subsystem's
    // do_assemble() is issued; then do_assemble_late() writes every one of
    // them (DiagInfo::hessians() is their range).
    virtual void do_report_late_extent(GlobalLinearSystem::LateExtentInfo& info);
    virtual void do_assemble_late(GlobalLinearSystem::DiagInfo& info);

    virtual Float do_diag_norm(GlobalLinearSystem::DiagNormInfo& info) = 0;
    virtual Float do_mass_norm(GlobalLinearSystem::DiagNormInfo& info) = 0;
    virtual U64   get_uid() const noexcept                             = 0;

  private:
    friend class GlobalLinearSystem;
    virtual void do_build() final override;

    void init();  // only be called by GlobalLinearSystem

    void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
    void receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info);

    void report_extent(GlobalLinearSystem::DiagExtentInfo& info);
    void prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info);
    void prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info);
    void assemble(GlobalLinearSystem::DiagInfo& info);
    void report_late_extent(GlobalLinearSystem::LateExtentInfo& info);
    void assemble_late(GlobalLinearSystem::DiagInfo& info);
    void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
    void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

    Float diag_norm(GlobalLinearSystem::DiagNormInfo& info);
    Float mass_norm(GlobalLinearSystem::DiagNormInfo& info);

    SizeT m_index = ~0ull;

    SimSystemSlot<GlobalLinearSystem> m_global_linear_system;
};
}  // namespace uipc::backend::cuda
