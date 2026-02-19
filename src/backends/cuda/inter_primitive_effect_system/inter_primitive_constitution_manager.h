#pragma once
#include <sim_system.h>
#include <utils/offset_count_collection.h>
#include <dytopo_effect_system/dytopo_effect_reporter.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::backend::cuda
{
class InterPrimitiveConstitution;

class InterPrimitiveConstitutionManager final : public DyTopoEffectReporter
{
  public:
    using DyTopoEffectReporter::DyTopoEffectReporter;

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
        friend class InterPrimitiveConstitutionManager;
        SizeT               m_index    = 0;
        const InterGeoInfo* m_geo_info = nullptr;
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SceneVisitor& scene, IndexT index)
            : m_impl(impl)
            , scene(&scene)
            , m_index(index)
        {
        }

        span<const InterGeoInfo>  inter_geo_infos() const noexcept;
        S<geometry::GeometrySlot> geo_slot(IndexT id) const;
        S<geometry::GeometrySlot> rest_geo_slot(IndexT id) const;


        template <typename ForEachGeometry>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ForEachGeometry&&               for_every_geometry) const;

      private:
        Impl*         m_impl  = nullptr;
        SceneVisitor* scene   = nullptr;
        IndexT        m_index = -1;
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

        muda::CBufferView<Vector3> positions() const noexcept;
        Float                      dt() const noexcept;

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
        friend class InterPrimitiveConstitutionManager;
        muda::BufferView<Float> m_energies;
    };

    class GradientHessianInfo : public BaseInfo
    {
      public:
        GradientHessianInfo(Impl*                             impl,
                            IndexT                            index,
                            bool                              gradient_only,
                            Float                             dt,
                            muda::DoubletVectorView<Float, 3> gradients,
                            muda::TripletMatrixView<Float, 3> hessians)
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
        friend class InterPrimitiveConstitutionManager;
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
        bool                              m_gradient_only = false;
    };

    class GradientHessianExtentInfo
    {
      public:
        bool gradient_only() const noexcept { return m_gradient_only; }
        void hessian_count(SizeT count) noexcept;
        void gradient_count(SizeT count) noexcept;

      private:
        friend class InterPrimitiveConstitution;
        friend class InterPrimitiveConstitutionManager;
        bool  m_gradient_only  = false;
        SizeT m_hessian_count  = 0;
        SizeT m_gradient_count = 0;
    };

    class EnergyExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept;

      private:
        friend class InterPrimitiveConstitutionManager;
        SizeT m_energy_count = 0;
    };

    class Impl
    {
      public:
        void init(SceneVisitor& scene);
        void compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info);
        void compute_gradient_hessian(GlobalDyTopoEffectManager::GradientHessianInfo& info);

        Float dt = 0.0;

        SimSystemSlotCollection<InterPrimitiveConstitution> constitutions;
        SimSystemSlot<GlobalVertexManager> global_vertex_manager;
        unordered_map<U64, IndexT>         uid_to_index;

        vector<InterGeoInfo>          inter_geo_infos;
        OffsetCountCollection<IndexT> constitution_geo_info_offsets_counts;

        OffsetCountCollection<IndexT> constitution_energy_offsets_counts;
        OffsetCountCollection<IndexT> constitution_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constitution_hessian_offsets_counts;
    };

  private:
    friend class SimEngine;
    virtual void do_build(DyTopoEffectReporter::BuildInfo& info) override;

    void do_init(DyTopoEffectReporter::InitInfo& info) override;

    friend class InterPrimitiveConstitution;
    void add_constitution(InterPrimitiveConstitution* constitution) noexcept;

    void do_report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info) override final;
    void do_report_gradient_hessian_extent(
        GlobalDyTopoEffectManager::GradientHessianExtentInfo& info) override final;
    void do_assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info) override;
    void do_compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info) override;

    virtual EnergyComponentFlags component_flags() override final
    {
        return EnergyComponentFlags::Complement;
    }

    Impl m_impl;

    template <typename ForEachGeometry>
    static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                          span<const InterGeoInfo>        geo_infos,
                          ForEachGeometry&&               for_every_geometry);
};
}  // namespace uipc::backend::cuda

#include "details/inter_primitive_constitution_manager.inl"