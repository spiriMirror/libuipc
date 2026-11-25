/*
 * NVIDIA contributions: side-stream and energy-count interfaces of the
 * animator constraints.
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
#include <animator/animator.h>
#include <finite_element/finite_element_method.h>
#include <line_search/line_searcher.h>
#include <utils/offset_count_collection.h>
#include <cuda_tool/cuda_tool.h>
#include <finite_element/fem_linear_subsystem_reporter.h>
#include <finite_element/fem_line_search_subreporter.h>

namespace uipc::backend::cuda
{
class FiniteElementConstraint;
class FiniteElementAnimator final : public Animator
{
  public:
    using Animator::Animator;

    using AnimatedGeoInfo = FiniteElementMethod::GeoInfo;

    class Impl;

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const AnimatedGeoInfo> anim_geo_infos() const;

        template <typename ForEach, typename ViewGetter>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&&                       for_each_action) noexcept;

      private:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, SizeT index, Float dt)
            : m_impl(impl)
            , m_index(index)
            , m_dt(dt)
        {
        }
        Float                           substep_ratio() const noexcept;
        Float                           dt() const noexcept;
        cuda_tool::CBufferView<Vector3> xs() const noexcept;
        cuda_tool::CBufferView<Vector3> x_prevs() const noexcept;
        cuda_tool::CBufferView<Float>   masses() const noexcept;
        cuda_tool::CBufferView<IndexT>  is_fixed() const noexcept;

      protected:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
        Float m_dt    = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl*                        impl,
                          SizeT                        index,
                          Float                        dt,
                          cuda_tool::BufferView<Float> energies,
                          cuda_tool::SideStreams*      side_streams = nullptr)
            : BaseInfo(impl, index, dt)
            , m_energies(energies)
            , m_stream(side_streams)
        {
        }

        cuda_tool::BufferView<Float> energies() const noexcept;
        // Stream for every launch of this constraint's energy.
        cudaStream_t stream() const noexcept { return m_stream.get(); }

      private:
        cuda_tool::BufferView<Float> m_energies;
        cuda_tool::LazySideStream    m_stream;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   SizeT index,
                                   Float dt,
                                   cuda_tool::DoubletVectorView<Float, 3> gradients,
                                   cuda_tool::TripletMatrixView<Float, 3> hessians,
                                   bool         gradient_only,
                                   cudaStream_t stream = nullptr)
            : BaseInfo(impl, index, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
            , m_stream(stream)
        {
        }
        cuda_tool::DoubletVectorView<Float, 3> gradients() const noexcept;
        cuda_tool::TripletMatrixView<Float, 3> hessians() const noexcept;
        bool                                   gradient_only() const noexcept;
        // Stream for every launch of the constraint's gradient and Hessian.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        cuda_tool::DoubletVectorView<Float, 3> m_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_hessians;
        bool                                   m_gradient_only = false;
        cudaStream_t                           m_stream        = nullptr;
    };

    class ReportExtentInfo
    {
      public:
        void hessian_count(SizeT count) noexcept;
        void gradient_count(SizeT count) noexcept;
        void energy_count(SizeT count) noexcept;
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class FiniteElementAnimator;
        friend class FiniteElementConstraint;
        SizeT        m_hessian_block_count    = 0;
        SizeT        m_gradient_segment_count = 0;
        SizeT        m_energy_count           = 0;
        bool         m_gradient_only          = false;
        mutable bool m_gradient_only_checked  = false;
    };

    class Impl
    {
      public:
        void init(backend::WorldVisitor& world);
        void step();

        FiniteElementMethod*       finite_element_method = nullptr;
        FiniteElementMethod::Impl& fem() const noexcept
        {
            return finite_element_method->m_impl;
        }

        GlobalAnimator* global_animator = nullptr;
        SimSystemSlotCollection<FiniteElementConstraint> constraints;
        unordered_map<U64, SizeT> uid_to_constraint_index;

        vector<AnimatedGeoInfo>       anim_geo_infos;
        OffsetCountCollection<IndexT> constraint_geo_info_offsets_counts;

        OffsetCountCollection<IndexT> constraint_energy_offsets_counts;
        OffsetCountCollection<IndexT> constraint_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constraint_hessian_offsets_counts;
    };

  private:
    friend class FiniteElementConstraint;
    void add_constraint(FiniteElementConstraint* constraint);  // only be called by FiniteElementConstraint

    friend class FiniteElementAnimatorLineSearchSubreporter;
    void compute_energy(FEMLineSearchSubreporter::ComputeEnergyInfo& info);  // only be called by FiniteElementAnimatorLineSearchSubreporter

    friend class FiniteElementAnimatorLinearSubsystemReporter;
    void assemble(FEMLinearSubsystemReporter::AssembleInfo& info);  // only be called by FEMLinearSubsystem

    Impl m_impl;

    virtual void               do_init() override;
    virtual void               do_step() override;
    virtual void               do_build(BuildInfo& info) override;
    virtual span<const IndexT> do_energy_counts() const noexcept override
    {
        return m_impl.constraint_energy_offsets_counts.counts();
    }
};
}  // namespace uipc::backend::cuda


#include "details/finite_element_animator.inl"