#pragma once
#include <sim_system.h>
#include <uipc/geometry/implicit_geometry_slot.h>
#include <muda/buffer/device_var.h>
namespace uipc::backend::cuda
{
class HalfPlane : public SimSystem
{
  public:
    static constexpr U64 ImplicitGeometryUID = 1ull;
    using SimSystem::SimSystem;

    using ImplicitGeometry = geometry::ImplicitGeometry;
    class Impl;

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _find_geometry(WorldVisitor& world);
        void _build_geometry();

        vector<ImplicitGeometry*> geos;

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

  protected:
    virtual void do_build() override;

  private:
    friend class HalfPlaneVertexReporter;
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
