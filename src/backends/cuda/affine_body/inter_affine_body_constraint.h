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
#include <sim_system.h>
#include <affine_body/inter_affine_body_animator.h>

namespace uipc::backend::cuda
{
class InterAffineBodyConstraint : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                            = 0;
    virtual U64  get_uid() const noexcept                             = 0;
    virtual void do_init(InterAffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_step(InterAffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) = 0;
    virtual void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) = 0;
    // True for a long gradient/Hessian chain (per-joint eigensolves): a
    // prelaunched assembly starts it first, on a stream of its own.
    virtual bool do_leads_assembly() const noexcept { return false; }


    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
    {
        InterAffineBodyAnimator::_for_each(geo_slots,
                                           animated_inter_geo_info(),
                                           std::forward<ForEachGeometry>(for_every_geometry));
    }

    span<const InterAffineBodyAnimator::AnimatedInterGeoInfo> animated_inter_geo_info() const noexcept;

  private:
    friend class InterAffineBodyAnimator;
    virtual void do_build() override final;

    void init(InterAffineBodyAnimator::FilteredInfo& info);
    void step(InterAffineBodyAnimator::FilteredInfo& info);
    void report_extent(InterAffineBodyAnimator::ReportExtentInfo& info);
    void compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info);
    void compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info);
    bool leads_assembly() const noexcept { return do_leads_assembly(); }

    SizeT                                  m_index = ~0ull;
    SimSystemSlot<InterAffineBodyAnimator> m_animator;
};
}  // namespace uipc::backend::cuda
