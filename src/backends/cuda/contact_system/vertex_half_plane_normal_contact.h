/*
 * NVIDIA contributions: side-stream assembly and energy interfaces.
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
#include <implicit_geometry/half_plane_vertex_reporter.h>

namespace uipc::backend::cuda
{
class GlobalTrajectoryFilter;
class VertexHalfPlaneTrajectoryFilter;

class VertexHalfPlaneNormalContact : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;

    class Impl;


    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular() const;
        cuda_tool::CBufferView<Vector2i>       PHs() const;
        cuda_tool::CBufferView<Vector3>        positions() const;
        cuda_tool::CBufferView<Vector3>        prev_positions() const;
        cuda_tool::CBufferView<Vector3>        rest_positions() const;
        cuda_tool::CBufferView<Float>          thicknesses() const;
        cuda_tool::CBufferView<IndexT>         contact_element_ids() const;
        cuda_tool::CBufferView<IndexT>         subscene_element_ids() const;
        Float                                  d_hat() const;
        cuda_tool::CBufferView<Float>          d_hats() const;
        Float                                  dt() const;
        Float                                  eps_velocity() const;
        IndexT                                 half_plane_vertex_offset() const;

      private:
        friend class VertexHalfPlaneNormalContact;
        Impl* m_impl;
    };

    class ContactInfo : public BaseInfo
    {
      public:
        ContactInfo(Impl* impl) noexcept
            : BaseInfo(impl)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradient_only() const noexcept { return m_gradient_only; }
        // Stream for every launch of the assembly (the legacy stream if null).
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class VertexHalfPlaneNormalContact;

        cuda_tool::DoubletVectorView<Float, 3> m_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_hessians;
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

        cuda_tool::BufferView<Float> energies() const noexcept;
        // Stream for every launch of the energy.
        cudaStream_t stream() const noexcept
        {
            return m_base ? m_base->stream() : cuda_tool::default_stream();
        }

      private:
        friend class VertexHalfPlaneNormalContact;
        const GlobalContactManager::EnergyInfo* m_base = nullptr;
        cuda_tool::BufferView<Float>            m_energies;
    };

    class Impl
    {
      public:
        void compute_energy(EnergyInfo& info);

        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;
        SimSystemSlot<GlobalContactManager>   global_contact_manager;
        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;
        SimSystemSlot<VertexHalfPlaneTrajectoryFilter> veretx_half_plane_trajectory_filter;
        SimSystemSlot<HalfPlaneVertexReporter> vertex_reporter;

        SizeT                                   PH_count = 0;
        S<const geometry::AttributeSlot<Float>> dt_attr;

        cuda_tool::CBufferView<Float>           energies;
        cuda_tool::CDoubletVectorView<Float, 3> gradients;
        cuda_tool::CTripletMatrixView<Float, 3> hessians;
    };

    cuda_tool::CBufferView<Vector2i>        PHs() const noexcept;
    cuda_tool::CBufferView<Float>           energies() const noexcept;
    cuda_tool::CDoubletVectorView<Float, 3> gradients() const noexcept;
    cuda_tool::CTripletMatrixView<Float, 3> hessians() const noexcept;

  protected:
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
