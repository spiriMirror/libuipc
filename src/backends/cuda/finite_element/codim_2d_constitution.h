/*
 * NVIDIA contributions: stream interfaces of the prelaunched FEM assembly.
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
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class Codim2DConstitution : public FiniteElementConstitution
{
  public:
    using FiniteElementConstitution::FiniteElementConstitution;

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(Codim2DConstitution* impl, SizeT index_in_dim, Float dt)
            : m_impl(impl)
            , m_index_in_dim(index_in_dim)
            , m_dt(dt)
        {
        }

        cuda_tool::CBufferView<Vector3>  xs() const noexcept;
        cuda_tool::CBufferView<Vector3>  x_bars() const noexcept;
        cuda_tool::CBufferView<Float>    rest_areas() const noexcept;
        cuda_tool::CBufferView<Float>    thicknesses() const noexcept;
        cuda_tool::CBufferView<Vector3i> indices() const noexcept;
        const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;
        Float dt() const noexcept;

      protected:
        SizeT                m_index_in_dim = ~0ull;
        Codim2DConstitution* m_impl         = nullptr;
        Float                m_dt           = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Codim2DConstitution*         impl,
                          SizeT                        index_in_dim,
                          Float                        dt,
                          cuda_tool::BufferView<Float> energies)
            : BaseInfo(impl, index_in_dim, dt)
            , m_energies(energies)
        {
        }

        auto energies() const noexcept { return m_energies; }

      private:
        cuda_tool::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Codim2DConstitution* impl,
                                   SizeT                index_in_dim,
                                   bool                 gradient_only,
                                   Float                dt,
                                   cuda_tool::DoubletVectorView<Float, 3> gradients,
                                   cuda_tool::TripletMatrixView<Float, 3> hessians,
                                   cudaStream_t stream = nullptr)
            : BaseInfo(impl, index_in_dim, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
            , m_stream(stream)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradient_only() const noexcept { return m_gradient_only; }
        // Stream for every launch of the gradient and Hessian.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        cuda_tool::DoubletVectorView<Float, 3> m_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_hessians;
        bool                                   m_gradient_only = false;
        cudaStream_t                           m_stream        = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;
    const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;

  private:
    friend class FiniteElementMethod;
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
    virtual void do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info) override final;
    virtual void do_compute_gradient_hessian(
        FiniteElementConstitution::ComputeGradientHessianInfo& info) override final;
    virtual IndexT get_dim() const noexcept override final;
};
}  // namespace uipc::backend::cuda
