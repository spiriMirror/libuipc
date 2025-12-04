#pragma once
#include <animator/animator.h>
#include <affine_body/affine_body_dynamics.h>
#include <line_search/line_searcher.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <affine_body/abd_linear_subsystem_reporter.h>
#include <affine_body/abd_line_search_subreporter.h>
#include <affine_body/inter_affine_body_constitution_manager.h>

namespace uipc::backend::cuda
{
class AffineBodyConstraint;
class ABDLineSearchReporter;
class ABDGradientHessianComputer;

class AffineBodyAnimator final : public Animator
{
  public:
    using Animator::Animator;

    class Impl;

    using AnimatedGeoInfo = AffineBodyDynamics::GeoInfo;

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const AnimatedGeoInfo> anim_geo_infos() const noexcept;

        template <typename ViewGetterF, typename ForEachF>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetterF&&                   getter,
                      ForEachF&&                      for_each);

        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_every_geometry);

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

        Float                                  substep_ratio() const noexcept;
        Float                                  dt() const noexcept;
        muda::CBufferView<Vector12>            qs() const noexcept;
        muda::CBufferView<Vector12>            q_prevs() const noexcept;
        muda::CBufferView<ABDJacobiDyadicMass> body_masses() const noexcept;
        muda::CBufferView<IndexT>              is_fixed() const noexcept;

      protected:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
        Float m_dt    = 0.0;
    };

    class EnergyInfo : public BaseInfo
    {
      public:
        EnergyInfo(Impl* impl, SizeT index, Float dt, muda::BufferView<Float> energy)
            : BaseInfo(impl, index, dt)
            , m_energies(energy)
        {
        }
        muda::BufferView<Float> energies() const noexcept;

      private:
        friend class AffineBodyAnimator;
        muda::BufferView<Float> m_energies;
    };

    class GradientHessianInfo : public BaseInfo
    {
      public:
        GradientHessianInfo(Impl*                              impl,
                            SizeT                              index,
                            Float                              dt,
                            muda::DoubletVectorView<Float, 12> gradients,
                            muda::TripletMatrixView<Float, 12> hessians)
            : BaseInfo(impl, index, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
        {
        }
        muda::DoubletVectorView<Float, 12> gradients() const noexcept;
        muda::TripletMatrixView<Float, 12> hessians() const noexcept;

      private:
        friend class AffineBodyAnimator;
        muda::DoubletVectorView<Float, 12> m_gradients;
        muda::TripletMatrixView<Float, 12> m_hessians;
    };

    class ReportExtentInfo
    {
      public:
        void hessian_block_count(SizeT count) noexcept;
        void gradient_segment_count(SizeT count) noexcept;
        void energy_count(SizeT count) noexcept;

      private:
        friend class AffineBodyAnimator;
        SizeT m_hessian_block_count    = 0;
        SizeT m_gradient_segment_count = 0;
        SizeT m_energy_count           = 0;
    };

    class Impl
    {
      public:
        void init(backend::WorldVisitor& world);
        void step();

        Float dt = 0.0;

        AffineBodyDynamics* affine_body_dynamics = nullptr;
        GlobalAnimator*     global_animator      = nullptr;
        SimSystemSlotCollection<AffineBodyConstraint> constraints;
        unordered_map<U64, SizeT>                     uid_to_constraint_index;

        vector<AnimatedGeoInfo>       anim_geo_infos;
        OffsetCountCollection<IndexT> constraint_geo_info_offsets_counts;

        OffsetCountCollection<IndexT> constraint_energy_offsets_counts;
        OffsetCountCollection<IndexT> constraint_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constraint_hessian_offsets_counts;
    };

  private:
    friend class AffineBodyConstraint;
    void add_constraint(AffineBodyConstraint* constraint);  // only be called by AffinieElementConstraint

    friend class AffineBodyAnimatorLineSearchSubreporter;
    void compute_energy(ABDLineSearchReporter::EnergyInfo& info);  // only be called by AffineBodyAnimatorLineSearchSubreporter

    friend class AffineBodyAnimatorLinearSubsystemReporter;
    void compute_gradient_hessian(ABDLinearSubsystem::AssembleInfo& info);  // only be called by AffineBodyAnimatorLinearSubsystemReporter

    Impl m_impl;

    virtual void do_init() override;
    virtual void do_step() override;
    virtual void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda

#include "details/affine_body_animator.inl"