#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <muda/ext/linear_system/triplet_matrix_view.h>
#include <muda/ext/linear_system/doublet_vector_view.h>
#include <utils/offset_count_collection.h>

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
        IndexT geo_id_to_geo_info_index(IndexT geo_id) const noexcept;
        IndexT body_id(geometry::GeometrySlot& slot);

      private:
        Impl*  m_impl  = nullptr;
        IndexT m_index = -1;
    };

    class EnergyExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept;

      private:
        friend class InterAffineBodyConstitutionManager;
        SizeT m_energy_count = 0;
    };

    class GradientHessianExtentInfo
    {
      public:
        void gradient_segment_count(SizeT count) noexcept;
        void hessian_block_count(SizeT count) noexcept;

      private:
        friend class InterAffineBodyConstitutionManager;
        SizeT m_gradient_count = 0;
        SizeT m_hessian_count  = 0;
    };

    class ComputeEnergyInfo
    {
      public:
        muda::BufferView<Float> energies();

      private:
        friend class InterAffineBodyConstitutionManager;
        muda::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo
    {
      public:
        muda::DoubletVectorView<Float, 12> gradients() noexcept;
        muda::TripletMatrixView<Float, 12> hessians() noexcept;

      private:
        friend class InterAffineBodyConstitutionManager;
        muda::DoubletVectorView<Float, 12> m_gradients;
        muda::TripletMatrixView<Float, 12> m_hessians;
    };

    class Impl
    {
      public:
        void init(SceneVisitor& scene);

        AffineBodyDynamics* affine_body_dynamics = nullptr;
        SimSystemSlotCollection<InterAffineBodyConstitution> constitutions;
        unordered_map<U64, IndexT>                           uid_to_index;

        vector<InterGeoInfo>          geo_infos;
        OffsetCountCollection<IndexT> constitution_geo_info_offsets_counts;
    };

  private:
    friend class SimEngine;
    virtual void do_build() override;

    void init();  // only be called by SimEngine

    friend class InterAffineBodyConstitution;
    void add_constitution(InterAffineBodyConstitution* constitution) noexcept;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda