/*
 * NVIDIA contributions: side-stream assembly and energy interfaces and the
 * prelaunched trial energy.
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
#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <contact_system/contact_coeff.h>
#include <collision_detection/simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
class SimplexNormalContact : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;
    constexpr static SizeT PTHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix
    constexpr static SizeT EEHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix
    constexpr static SizeT PEHalfHessianSize = 3 * (3 + 1) / 2;  // 3 vertices, symmetric matrix
    constexpr static SizeT PPHalfHessianSize = 2 * (2 + 1) / 2;  // 2 vertices, symmetric matrix

    class Impl;

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular() const;
        cuda_tool::CBufferView<Vector4i>       PTs() const;
        cuda_tool::CBufferView<Vector4i>       EEs() const;
        cuda_tool::CBufferView<Vector3i>       PEs() const;
        cuda_tool::CBufferView<Vector2i>       PPs() const;

        cuda_tool::CBufferView<Float>   thicknesses() const;
        cuda_tool::CBufferView<Vector3> positions() const;
        cuda_tool::CBufferView<Vector3> prev_positions() const;
        cuda_tool::CBufferView<Vector3> rest_positions() const;
        cuda_tool::CBufferView<IndexT>  contact_element_ids() const;
        Float                           d_hat() const;
        cuda_tool::CBufferView<Float>   d_hats() const;
        Float                           dt() const;
        Float                           eps_velocity() const;

      private:
        friend class SimplexNormalContact;
        Impl* m_impl;
    };

    class ContactInfo : public BaseInfo
    {
      public:
        ContactInfo(Impl* impl) noexcept
            : BaseInfo(impl)
        {
        }
        auto PT_gradients() const noexcept { return m_PT_gradients; }
        auto PT_hessians() const noexcept { return m_PT_hessians; }
        auto EE_gradients() const noexcept { return m_EE_gradients; }
        auto EE_hessians() const noexcept { return m_EE_hessians; }
        auto PE_gradients() const noexcept { return m_PE_gradients; }
        auto PE_hessians() const noexcept { return m_PE_hessians; }
        auto PP_gradients() const noexcept { return m_PP_gradients; }
        auto PP_hessians() const noexcept { return m_PP_hessians; }
        bool gradient_only() const noexcept { return m_gradient_only; }
        // Stream for every launch of the assembly (the legacy stream if null).
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class SimplexNormalContact;
        cuda_tool::DoubletVectorView<Float, 3> m_PT_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PT_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_EE_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_EE_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_PE_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PE_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_PP_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PP_hessians;
        bool                                   m_gradient_only = false;
        cudaStream_t                           m_stream        = nullptr;
    };

    class BuildInfo
    {
      public:
    };

    class EnergyInfo : public BaseInfo
    {
      public:
        EnergyInfo(Impl* impl, const GlobalContactManager::EnergyInfo* base = nullptr) noexcept
            : BaseInfo(impl)
            , m_base(base)
        {
        }

        // Stream for every launch of the energy.
        cudaStream_t stream() const noexcept
        {
            return m_base ? m_base->stream() : cuda_tool::default_stream();
        }

        cuda_tool::BufferView<Float> PT_energies() const noexcept
        {
            return m_PT_energies;
        }
        cuda_tool::BufferView<Float> EE_energies() const noexcept
        {
            return m_EE_energies;
        }
        cuda_tool::BufferView<Float> PE_energies() const noexcept
        {
            return m_PE_energies;
        }
        cuda_tool::BufferView<Float> PP_energies() const noexcept
        {
            return m_PP_energies;
        }
        // PT | EE | PE | PP energies, contiguous
        cuda_tool::BufferView<Float> energies() const noexcept
        {
            return m_energies;
        }
        // A trial energy launched before the read-back of the trial's
        // active-set counts (see the protected interface): energies() spans
        // the capacity of the pending active set, and the per-type energies
        // are empty.
        bool prelaunch() const noexcept { return m_prelaunch; }

      private:
        friend class SimplexNormalContact;
        const GlobalContactManager::EnergyInfo* m_base = nullptr;
        cuda_tool::BufferView<Float>            m_PT_energies;
        cuda_tool::BufferView<Float>            m_EE_energies;
        cuda_tool::BufferView<Float>            m_PE_energies;
        cuda_tool::BufferView<Float>            m_PP_energies;
        cuda_tool::BufferView<Float>            m_energies;
        bool                                    m_prelaunch = false;
    };

    class Impl
    {
      public:
        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;
        SimSystemSlot<GlobalContactManager>   global_contact_manager;
        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;

        SimSystemSlot<SimplexTrajectoryFilter> simplex_trajectory_filter;

        // constraint count

        SizeT PT_count = 0;
        SizeT EE_count = 0;
        SizeT PE_count = 0;
        SizeT PP_count = 0;

        S<const geometry::AttributeSlot<Float>> dt_attr;
        cuda_tool::DeviceVar<IndexT>            selected_count;

        Float reserve_ratio = 1.1;

        template <typename T>
        void loose_resize(cuda_tool::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }

        cuda_tool::CBufferView<Float>           PT_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PT_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PT_hessians;

        cuda_tool::CBufferView<Float>           EE_energies;
        cuda_tool::CDoubletVectorView<Float, 3> EE_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> EE_hessians;

        cuda_tool::CBufferView<Float>           PE_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PE_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PE_hessians;

        cuda_tool::CBufferView<Float>           PP_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PP_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PP_hessians;
    };

    cuda_tool::CBufferView<Vector4i>        PTs() const;
    cuda_tool::CBufferView<Float>           PT_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PT_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PT_hessians() const;

    cuda_tool::CBufferView<Vector4i>        EEs() const;
    cuda_tool::CBufferView<Float>           EE_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> EE_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> EE_hessians() const;

    cuda_tool::CBufferView<Vector3i>        PEs() const;
    cuda_tool::CBufferView<Float>           PE_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PE_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PE_hessians() const;

    cuda_tool::CBufferView<Vector2i>        PPs() const;
    cuda_tool::CBufferView<Float>           PP_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PP_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PP_hessians() const;

    // The active set of the trial's pending filter_active (see
    // SimplexTrajectoryFilter::pending_active_set).
    const SimplexTrajectoryFilter::PendingActiveSet& pending_active_set() const;

  protected:
    // A subclass whose do_compute_energy launches with
    // EnergyInfo::prelaunch() by the pending active set (its lists at
    // capacity, its counts on the device) may override
    // do_report_prelaunch_energy_extent and report the pending count of
    // their capacity.
    virtual void do_build(BuildInfo& info)           = 0;
    virtual void do_compute_energy(EnergyInfo& info) = 0;
    virtual void do_assemble(ContactInfo& info)      = 0;

  private:
    virtual void do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info) override final;
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) override final;
    virtual void do_report_gradient_hessian_extent(
        GlobalContactManager::GradientHessianExtentInfo& info) override final;
    virtual void do_assemble(GlobalContactManager::GradientHessianInfo& info) override final;
    virtual void do_build(ContactReporter::BuildInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
