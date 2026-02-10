#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <muda/ext/linear_system/triplet_matrix_view.h>
#include <muda/ext/linear_system/doublet_vector_view.h>
#include <utils/offset_count_collection.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/abd_linear_subsystem.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::backend::cuda
{
class InterAffineBodyConstitution;
class InterAffineBodyConstitutionManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class InterGeoInfo
    {
      public:
        IndexT geo_slot_index   = -1;
        IndexT geo_id           = -1;
        U64    constitution_uid = 0;  // unique identifier for the constitution
    };

    class ForEachInfo
    {
      public:
        SizeT               index() const noexcept { return m_index; }
        const InterGeoInfo& geo_info() const noexcept { return *m_geo_info; }

      private:
        friend class InterAffineBodyConstitutionManager;
        SizeT               m_index    = 0;
        const InterGeoInfo* m_geo_info = nullptr;
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, IndexT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const InterGeoInfo> inter_geo_infos() const noexcept;
        span<const AffineBodyDynamics::GeoInfo> geo_infos() const noexcept;
        const AffineBodyDynamics::GeoInfo& geo_info(IndexT geo_id) const noexcept;
        IndexT body_id(IndexT geo_id) const noexcept;
        IndexT body_id(IndexT geo_id, IndexT instance_id) const noexcept;
        geometry::SimplicialComplex* body_geo(span<S<geometry::GeometrySlot>> geo_slots,
                                              IndexT geo_id) const noexcept;
        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_every_geometry) const;

      private:
        Impl*  m_impl  = nullptr;
        IndexT m_index = -1;
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
        EnergyInfo(Impl* impl, IndexT index, Float dt, muda::BufferView<Float> energy)
            : BaseInfo(impl, index, dt)
            , m_energies(energy)
        {
        }
        muda::BufferView<Float> energies() const noexcept;

      private:
        friend class InterAffineBodyConstitutionManager;
        muda::BufferView<Float> m_energies;
    };

    class GradientHessianInfo : public BaseInfo
    {
      public:
        GradientHessianInfo(Impl*                              impl,
                            IndexT                             index,
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
        friend class InterAffineBodyConstitutionManager;
        muda::DoubletVectorView<Float, 12> m_gradients;
        muda::TripletMatrixView<Float, 12> m_hessians;
        bool                               m_gradient_only = false;
    };

    class GradientHessianExtentInfo
    {
      public:
        bool gradient_only() const noexcept { return m_gradient_only; }
        void hessian_count(SizeT count) noexcept;
        void gradient_count(SizeT count) noexcept;

      private:
        friend class InterAffineBodyConstitution;
        friend class InterAffineBodyConstitutionManager;
        bool  m_gradient_only  = false;
        SizeT m_hessian_count  = 0;
        SizeT m_gradient_count = 0;
    };

    class EnergyExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept;

      private:
        friend class InterAffineBodyConstitutionManager;
        SizeT m_energy_count = 0;
    };

    class Impl
    {
      public:
        void init(SceneVisitor& scene);
        void report_energy_extent(ABDLineSearchReporter::ReportExtentInfo& info);
        void compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info);
        void report_gradient_hessian_extent(ABDLinearSubsystem::ReportExtentInfo& info);
        void compute_gradient_hessian(ABDLinearSubsystem::AssembleInfo& info);

        Float dt = 0.0;

        AffineBodyDynamics* affine_body_dynamics = nullptr;
        SimSystemSlotCollection<InterAffineBodyConstitution> constitutions;
        unordered_map<U64, IndexT>                           uid_to_index;

        vector<InterGeoInfo>          inter_geo_infos;
        OffsetCountCollection<IndexT> constitution_geo_info_offsets_counts;

        OffsetCountCollection<IndexT> constitution_energy_offsets_counts;
        OffsetCountCollection<IndexT> constitution_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constitution_hessian_offsets_counts;
    };

  private:
    friend class SimEngine;
    virtual void do_build() override;

    void init();  // only be called by SimEngine

    friend class InterAffineBodyConstitution;
    void add_constitution(InterAffineBodyConstitution* constitution) noexcept;

    friend class InterAffineBodyConstitutionLineSearchSubreporter;
    friend class InterAffineBodyConstitutionLinearSubsystemReporter;
    friend class InterAffineBodyAnimator;

    Impl m_impl;

    template <typename ForEachGeometry>
    static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                          span<const InterGeoInfo>        geo_infos,
                          ForEachGeometry&&               for_every_geometry);
};
}  // namespace uipc::backend::cuda

#include "details/inter_affine_body_constitution_manager.inl"