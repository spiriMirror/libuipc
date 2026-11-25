/*
 * NVIDIA contributions: the prelaunched energy extent and one-warp
 * assembly-block report interfaces.
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
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <energy_component_flags.h>

namespace uipc::backend::cuda
{
class DyTopoEffectReporter : public SimSystem
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

    class Impl
    {
      public:
        cuda_tool::CBufferView<Float>           energies;
        cuda_tool::CDoubletVectorView<Float, 3> gradients;
        cuda_tool::CTripletMatrixView<Float, 3> hessians;
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo&);
    virtual void do_report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info) = 0;
    virtual void do_report_gradient_hessian_extent(
        GlobalDyTopoEffectManager::GradientHessianExtentInfo& info) = 0;
    virtual void do_assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info) = 0;
    virtual void do_compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info) = 0;
    // Optional: report the extent of a prelaunched trial energy and return
    // true; the energy is then launched with EnergyInfo::prelaunch() and,
    // after the count read-back, taken with EnergyInfo::prelaunched().
    virtual bool do_report_prelaunch_energy_extent(GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo& info);
    // Optional: the one-warp blocks of the latency-bound assembly kernels the
    // next do_assemble() launches for the current active set (default 0).
    virtual SizeT                do_report_assembly_blocks();
    virtual EnergyComponentFlags component_flags() = 0;

  private:
    friend class GlobalDyTopoEffectManager;
    friend class DyTopoEffectLineSearchReporter;
    void init();  // only be called by GlobalDyTopoEffectManager
    void do_build() final override;
    void report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info);
    bool report_prelaunch_energy_extent(GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo& info);
    SizeT report_assembly_blocks();
    void report_gradient_hessian_extent(GlobalDyTopoEffectManager::GradientHessianExtentInfo& info);
    void  assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info);
    void  compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info);
    SizeT m_index = ~0ull;
    Impl  m_impl;
};
}  // namespace uipc::backend::cuda
