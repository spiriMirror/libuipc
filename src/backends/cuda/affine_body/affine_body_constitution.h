/*
 * NVIDIA contributions: side-stream interfaces for the prelaunched assembly
 * and the line-search energy.
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
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/abd_linear_subsystem.h>

namespace uipc::backend::cuda
{
class AffineBodyConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    U64 uid() const;

    class BuildInfo
    {
      public:
    };

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
        AffineBodyDynamics::Impl&         abd() noexcept
        {
            return affine_body_dynamics->m_impl;
        }
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, IndexT index) noexcept
            : m_impl(impl)
            , m_index(index)
        {
        }

        auto is_fixed() const noexcept
        {
            return m_impl->affine_body_dynamics->body_is_fixed();
        }

        cuda_tool::CBufferView<Vector12> qs() const noexcept;

        cuda_tool::CBufferView<Vector12> q_prevs() const noexcept;

        cuda_tool::CBufferView<Float> volumes() const noexcept;

      protected:
        Impl*  m_impl  = nullptr;
        IndexT m_index = -1;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl*  impl,
                          IndexT index,
                          ABDLineSearchReporter::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl, index)
            , base_info(base_info)
        {
        }

        auto energies() const noexcept { return base_info->energies(); }

        auto dt() const noexcept { return base_info->dt(); }

        // Every launch of the constitution's energy goes to this stream.
        auto stream() const noexcept { return base_info->stream(); }

      private:
        ABDLineSearchReporter::ComputeEnergyInfo* base_info = nullptr;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl*  impl,
                                   IndexT index,
                                   ABDLinearSubsystem::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl, index)
            , base_info(base_info)
        {
        }

        auto gradient_only() const noexcept
        {
            return base_info->gradient_only();
        }
        auto gradients() const noexcept { return base_info->gradients(); }
        auto hessians() const noexcept { return base_info->hessians(); }
        auto dt() const noexcept { return base_info->dt(); }
        auto stream() const noexcept { return base_info->stream(); }

      private:
        ABDLinearSubsystem::ComputeGradientHessianInfo* base_info = nullptr;
    };

  protected:
    virtual U64  get_uid() const                                 = 0;
    virtual void do_build(BuildInfo& info)                       = 0;
    virtual void do_init(AffineBodyDynamics::FilteredInfo& info) = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info)      = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    friend class AffineBodyDynamics;
    friend class ABDLineSearchReporter;
    friend class ABDLinearSubsystem;

    virtual void do_build() override final;

    void init(AffineBodyDynamics::FilteredInfo& info);
    void compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info);
    void compute_gradient_hessian(ABDLinearSubsystem::ComputeGradientHessianInfo& info);

    IndexT m_index = -1;
    Impl   m_impl;
};
}  // namespace uipc::backend::cuda
