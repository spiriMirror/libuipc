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
class InterAffineBodyConstraint;
class ABDLineSearchReporter;
class ABDGradientHessianComputer;

class InterAffineBodyAnimator final : public Animator
{
  public:
    using Animator::Animator;

    class Impl;

    using AnimatedInterGeoInfo = InterAffineBodyConstitutionManager::InterGeoInfo;
    using ForEachInfo  = InterAffineBodyConstitutionManager::ForEachInfo;
    using InterGeoInfo = InterAffineBodyConstitutionManager::InterGeoInfo;

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const AnimatedInterGeoInfo> anim_inter_geo_infos() const noexcept;
        span<const AffineBodyDynamics::GeoInfo> geo_infos() const noexcept;
        const AffineBodyDynamics::GeoInfo& geo_info(IndexT geo_id) const noexcept;
        IndexT body_id(IndexT geo_id) const noexcept;
        IndexT body_id(IndexT geo_id, IndexT instance_id) const noexcept;
        geometry::SimplicialComplex* body_geo(span<S<geometry::GeometrySlot>> geo_slots,
                                              IndexT geo_id) const noexcept;
        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
        {
            InterAffineBodyAnimator::_for_each(geo_slots,
                                               this->anim_inter_geo_infos(),
                                               std::forward<ForEachGeometry>(for_every_geometry));
        }

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

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, SizeT index, Float dt, muda::BufferView<Float> energy)
            : BaseInfo(impl, index, dt)
            , m_energies(energy)
        {
        }
        muda::BufferView<Float> energies() const noexcept;

      private:
        friend class InterAffineBodyAnimator;
        muda::BufferView<Float> m_energies;
    };

    class GradientHessianInfo : public BaseInfo
    {
      public:
        GradientHessianInfo(Impl*                              impl,
                            SizeT                              index,
                            Float                              dt,
                            muda::DoubletVectorView<Float, 12> gradients,
                            muda::TripletMatrixView<Float, 12> hessians,
                            bool                               gradient_only)
            : BaseInfo(impl, index, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
        {
        }
        muda::DoubletVectorView<Float, 12> gradients() const noexcept;
        muda::TripletMatrixView<Float, 12> hessians() const noexcept;
        bool                               gradient_only() const noexcept;

      private:
        friend class InterAffineBodyAnimator;
        muda::DoubletVectorView<Float, 12> m_gradients;
        muda::TripletMatrixView<Float, 12> m_hessians;
        bool                               m_gradient_only = false;
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
        friend class InterAffineBodyAnimator;
        friend class InterAffineBodyConstraint;
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

        Float dt = 0.0;

        AffineBodyDynamics*                 affine_body_dynamics = nullptr;
        InterAffineBodyConstitutionManager* manager              = nullptr;
        GlobalAnimator*                     global_animator      = nullptr;

        SimSystemSlotCollection<InterAffineBodyConstraint> constraints;
        unordered_map<U64, SizeT> uid_to_constraint_index;

        vector<AnimatedInterGeoInfo>  anim_geo_infos;
        OffsetCountCollection<IndexT> constraint_geo_info_offsets_counts;

        // Constraints

        OffsetCountCollection<IndexT> constraint_energy_offsets_counts;
        OffsetCountCollection<IndexT> constraint_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constraint_hessian_offsets_counts;
    };

    template <typename ForEachGeometry>
    static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                          span<const InterGeoInfo>        geo_infos,
                          ForEachGeometry&&               for_every_geometry)
    {
        InterAffineBodyConstitutionManager::_for_each(geo_slots, geo_infos, for_every_geometry);
    }

  private:
    friend class InterAffineBodyConstraint;
    void add_constraint(InterAffineBodyConstraint* constraint);  // only be called by AffineBodyConstraint

    friend class InterAffineBodyAnimatorLineSearchSubreporter;
    void compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info);  // only be called by AffineBodyAnimatorLineSearchSubreporter

    friend class InterAffineBodyAnimatorLinearSubsystemReporter;
    void compute_gradient_hessian(ABDLinearSubsystem::AssembleInfo& info);  // only be called by AffineBodyAnimatorLinearSubsystemReporter

    Impl m_impl;

    virtual void do_init() override;
    virtual void do_step() override;
    virtual void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda