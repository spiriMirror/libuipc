/*
 * NVIDIA contributions: stream and direct-gradient interfaces of the
 * prelaunched FEM assembly and the line-search energy; the grid cap of the
 * gradient and Hessian launch.
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
#include <cuda_tool/cuda_tool.h>
#include <finite_element/finite_element_method.h>
#include <utils/offset_count_collection.h>
#include <finite_element/fem_linear_subsystem.h>
#include <finite_element/fem_line_search_reporter.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;

class FiniteElementElastics final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class ReportExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept { m_energy_count = count; }
        void gradient_count(SizeT count) noexcept { m_gradient_count = count; }
        void hessian_count(SizeT count) noexcept { m_hessian_count = count; }
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class FiniteElementElastics;
        friend class FiniteElementConstitution;
        friend class FiniteElementExtraConstitution;

        SizeT        m_energy_count          = 0;
        SizeT        m_gradient_count        = 0;
        SizeT        m_hessian_count         = 0;
        bool         m_gradient_only         = false;
        mutable bool m_gradient_only_checked = false;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Impl*                        impl,
                          SizeT                        index,
                          Float                        dt,
                          cuda_tool::BufferView<Float> energies,
                          cuda_tool::SideStreams*      side_streams = nullptr)
            : m_impl(impl)
            , m_index(index)
            , m_dt(dt)
            , m_energies(energies)
            , m_stream(side_streams)
        {
        }

        auto energies() const noexcept
        {
            auto [offset, count] = m_impl->constitution_energy_offsets_counts[m_index];
            return m_energies.subview(offset, count);
        }
        auto dt() const noexcept { return m_dt; }
        // Stream for every launch of this constitution's energy.
        cudaStream_t stream() const noexcept { return m_stream.get(); }

      private:
        Impl*                        m_impl  = nullptr;
        SizeT                        m_index = 0;
        Float                        m_dt    = 0.0;
        cuda_tool::BufferView<Float> m_energies;
        cuda_tool::LazySideStream    m_stream;
    };

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   SizeT index,
                                   bool  gradient_only,
                                   Float dt,
                                   cuda_tool::DoubletVectorView<Float, 3> gradients,
                                   cuda_tool::TripletMatrixView<Float, 3> hessians,
                                   cudaStream_t stream = nullptr)
            : m_impl(impl)
            , m_index(index)
            , m_gradient_only(gradient_only)
            , m_dt(dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_stream(stream)
        {
        }

        auto gradient_only() const noexcept { return m_gradient_only; }
        cuda_tool::DoubletVectorView<Float, 3> gradients() const noexcept;
        cuda_tool::TripletMatrixView<Float, 3> hessians() const noexcept;

        auto dt() const noexcept { return m_dt; }
        // Stream for every launch of the constitution's gradient and Hessian.
        cudaStream_t stream() const noexcept { return m_stream; }

        // Direct gradients (FEMLinearSubsystem::AssembleInfo): a constitution
        // that adds its gradient to direct_gradients() itself calls
        // gradients_added_directly(); the doublets of the others are added.
        bool direct_gradients_available() const noexcept
        {
            return m_assemble && m_assemble->direct_gradients_available();
        }
        cuda_tool::DenseVectorView<Float> direct_gradients() const noexcept
        {
            return m_assemble->direct_gradients();
        }
        cuda_tool::CBufferView<IndexT> is_fixed() const noexcept
        {
            return m_assemble->is_fixed();
        }
        void gradients_added_directly() const noexcept
        {
            m_added_directly = true;
        }
        // Grid cap (FEMLinearSubsystem::AssembleInfo::block_cap(), 0: none).
        int block_cap() const noexcept
        {
            return m_assemble ? m_assemble->block_cap() : 0;
        }

      private:
        friend class FiniteElementElastics;
        Impl*                                   m_impl          = nullptr;
        SizeT                                   m_index         = 0;
        bool                                    m_gradient_only = false;
        Float                                   m_dt            = 0.0;
        cuda_tool::DoubletVectorView<Float, 3>  m_gradients;
        cuda_tool::TripletMatrixView<Float, 3>  m_hessians;
        cudaStream_t                            m_stream         = nullptr;
        const FEMLinearSubsystem::AssembleInfo* m_assemble       = nullptr;
        mutable bool                            m_added_directly = false;
    };

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> finite_element_method;
        FiniteElementMethod::Impl&         fem()
        {
            return finite_element_method->m_impl;
        }

        OffsetCountCollection<IndexT> constitution_energy_offsets_counts;
        OffsetCountCollection<IndexT> constitution_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constitution_hessian_offsets_counts;

        void assemble(FEMLinearSubsystem::AssembleInfo& info);
        void compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info);
    };

  private:
    friend class FiniteElementElasticsLinearSubsystemReporter;
    friend class FiniteElementElasticsLineSearchSubreporter;

    virtual void do_build() override;

    friend class FiniteElementMethod;
    void init();  // only be called in FiniteElementMethod

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
