#pragma once
#include <sim_system.h>
#include <uipc/geometry/implicit_geometry_slot.h>
#include <muda/buffer/device_var.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class HalfPlane : public SimSystem
{
  public:
    static constexpr U64 ImplicitGeometryUID = 1ull;
    using SimSystem::SimSystem;

    using ImplicitGeometry = geometry::ImplicitGeometry;

    class GeoInfo
    {
      public:
        IndexT geo_slot_index = -1;
        IndexT geo_id         = -1;
        IndexT vertex_offset  = -1;
        IndexT vertex_count   = 0;
    };

    class ForEachInfo
    {
      public:
        SizeT          global_index() const noexcept { return m_global_index; }
        const GeoInfo& geo_info() const noexcept { return *m_geo_info; }

      private:
        friend class HalfPlane;
        SizeT          m_global_index = 0;
        const GeoInfo* m_geo_info     = nullptr;
    };

    class Impl;


    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _find_geometry(WorldVisitor& world);
        void _build_geometry();

        vector<GeoInfo>               geo_infos;
        vector<ImplicitGeometry*>     geos;
        OffsetCountCollection<IndexT> geo_vertex_offset_count;


        vector<IndexT>  h_contact_ids;
        vector<Vector3> h_normals;
        vector<Vector3> h_positions;

        muda::DeviceBuffer<Vector3> normals;
        muda::DeviceBuffer<Vector3> positions;
        muda::DeviceBuffer<IndexT>  contact_ids;
    };

    muda::CBufferView<Vector3> normals() const;
    muda::CBufferView<Vector3> positions() const;
    muda::CBufferView<IndexT>  contact_ids() const;

    span<const GeoInfo> geo_infos() const noexcept { return m_impl.geo_infos; }

    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
    {
        _for_each(geo_slots, m_impl.geo_infos, std::forward<ForEachGeometry>(for_every_geometry));
    }

  protected:
    virtual void do_build() override;

  private:
    friend class HalfPlaneVertexReporter;
    friend class HalfPlaneBodyReporter;
    Impl m_impl;

    template <typename ForEachGeometry>
    static void _for_each(span<S<geometry::GeometrySlot>> geo_slots,
                          span<GeoInfo>                   geo_infos,
                          ForEachGeometry&&               for_every_geometry);
};
}  // namespace uipc::backend::cuda

#include "details/half_plane.inl"