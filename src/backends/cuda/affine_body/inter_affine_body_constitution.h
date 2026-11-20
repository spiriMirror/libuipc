/*
 * NVIDIA contributions: the leading-producer interface.
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
#include <affine_body/inter_affine_body_constitution_manager.h>

namespace uipc::backend::cuda
{
class InterAffineBodyConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    class BuildInfo
    {
      public:
    };

    using InterGeoInfo = InterAffineBodyConstitutionManager::InterGeoInfo;
    using FilteredInfo = InterAffineBodyConstitutionManager::FilteredInfo;
    using EnergyExtentInfo = InterAffineBodyConstitutionManager::EnergyExtentInfo;
    using ComputeEnergyInfo = InterAffineBodyConstitutionManager::EnergyInfo;
    using GradientHessianExtentInfo =
        InterAffineBodyConstitutionManager::GradientHessianExtentInfo;
    using ComputeGradientHessianInfo = InterAffineBodyConstitutionManager::GradientHessianInfo;

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)   = 0;
    virtual void do_init(FilteredInfo& info) = 0;

    virtual void do_report_energy_extent(EnergyExtentInfo& info) = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info)      = 0;

    virtual void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

    virtual U64 get_uid() const noexcept = 0;  // unique identifier for this constitution

    // True for a long gradient/Hessian chain (per-joint eigensolves): a
    // prelaunched assembly starts it first, on a stream of its own.
    virtual bool do_leads_assembly() const noexcept { return false; }

    span<const InterGeoInfo> inter_geo_info() const noexcept;

    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
    {
        InterAffineBodyConstitutionManager::_for_each(
            geo_slots, inter_geo_info(), std::forward<ForEachGeometry>(for_every_geometry));
    }

  private:
    friend class InterAffineBodyConstitutionManager;
    virtual void do_build() override;

    void init(FilteredInfo& info);
    void report_energy_extent(EnergyExtentInfo& info);
    void compute_energy(ComputeEnergyInfo& info);

    void report_gradient_hessian_extent(GradientHessianExtentInfo& info);
    void compute_gradient_hessian(ComputeGradientHessianInfo& info);
    bool leads_assembly() const noexcept { return do_leads_assembly(); }

    IndexT m_index = -1;  // index in the InterAffineBodyConstitutionManager
    SimSystemSlot<InterAffineBodyConstitutionManager> m_manager;
};
}  // namespace uipc::backend::cuda