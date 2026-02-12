#pragma once
#include <animator/animator.h>
#include <finite_element/finite_element_method.h>
#include <line_search/line_searcher.h>
#include <utils/offset_count_collection.h>
#include <muda/ext/linear_system/device_dense_vector.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
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
        Float                      substep_ratio() const noexcept;
        Float                      dt() const noexcept;
        muda::CBufferView<Vector3> xs() const noexcept;
        muda::CBufferView<Vector3> x_prevs() const noexcept;
        muda::CBufferView<Float>   masses() const noexcept;
        muda::CBufferView<IndexT>  is_fixed() const noexcept;

      protected:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
        Float m_dt    = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, SizeT index, Float dt, muda::BufferView<Float> energies)
            : BaseInfo(impl, index, dt)
            , m_energies(energies)
        {
        }

        muda::BufferView<Float> energies() const noexcept;

      private:
        muda::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl*                             impl,
                                   SizeT                             index,
                                   Float                             dt,
                                   muda::DoubletVectorView<Float, 3> gradients,
                                   muda::TripletMatrixView<Float, 3> hessians,
                                   bool                              gradient_only)
            : BaseInfo(impl, index, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
        {
        }
        muda::DoubletVectorView<Float, 3> gradients() const noexcept;
        muda::TripletMatrixView<Float, 3> hessians() const noexcept;
        bool                              gradient_only() const noexcept;

      private:
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
        bool                              m_gradient_only = false;
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
        SizeT m_hessian_block_count    = 0;
        SizeT m_gradient_segment_count = 0;
        SizeT m_energy_count           = 0;
        bool  m_gradient_only          = false;
        mutable bool m_gradient_only_checked = false;
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

    virtual void do_init() override;
    virtual void do_step() override;
    virtual void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda


#include "details/finite_element_animator.inl"