/*
 * NVIDIA contributions: stream interfaces of the prelaunched FEM assembly and
 * the line-search energy.
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
#include <finite_element/finite_element_method.h>
#include <finite_element/fem_line_search_reporter.h>
#include <finite_element/fem_linear_subsystem.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> finite_element_method;
    };

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        auto is_fixed() const noexcept
        {
            return m_impl->finite_element_method->is_fixed();
        }

        auto is_dynamic() const noexcept
        {
            return m_impl->finite_element_method->is_dynamic();
        }

        auto x_bars() const noexcept
        {
            return m_impl->finite_element_method->x_bars();
        }

        auto xs() const noexcept { return m_impl->finite_element_method->xs(); }

        auto x_tildes() const noexcept
        {
            return m_impl->finite_element_method->x_tildes();
        }

        auto x_prevs() const noexcept
        {
            return m_impl->finite_element_method->x_prevs();
        }

        auto masses() const noexcept
        {
            return m_impl->finite_element_method->masses();
        }


      protected:
        Impl* m_impl = nullptr;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, FEMLineSearchReporter::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto energies() const noexcept { return base_info->energies(); }
        auto dt() const noexcept { return base_info->dt(); }
        // Every launch of the kinetic energy goes to this stream.
        auto stream() const noexcept { return base_info->stream(); }

      private:
        FEMLineSearchReporter::ComputeEnergyInfo* base_info = nullptr;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   FEMLinearSubsystem::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl)
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
        cudaStream_t stream() const noexcept { return base_info->stream(); }

      private:
        FEMLinearSubsystem::ComputeGradientHessianInfo* base_info = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    virtual void do_build() override final;

    friend class FEMLineSearchReporter;
    void compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info);

    friend class FEMLinearSubsystem;
    void compute_gradient_hessian(FEMLinearSubsystem::ComputeGradientHessianInfo& info);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
