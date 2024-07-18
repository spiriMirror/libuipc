#pragma once
#include <sim_system.h>
#include <uipc/geometry/implicit_geometry_slot.h>
#include <muda/buffer/device_var.h>
namespace uipc::backend::cuda
{
class Ground : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    using ImplicitGeometry = geometry::ImplicitGeometry;
    class Impl;

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void _find_geometry(WorldVisitor& world);
        void _build_geometry();

        ImplicitGeometry* ground_geo;

        Vector3 normal;
        Vector3 position;
    };

    //const Vector3& normal() const;
    //const Vector3& position() const;

  protected:
    virtual void do_build() override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
