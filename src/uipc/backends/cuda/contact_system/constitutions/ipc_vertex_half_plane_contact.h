#pragma once
#include <contact_system/vertex_half_plane_contact_constitution.h>
#include <implicit_geometry/half_plane.h>
namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneContact final : public VertexHalfPlaneContactConstitution
{
  public:
    using VertexHalfPlaneContactConstitution::VertexHalfPlaneContactConstitution;

    virtual void do_build(BuildInfo& info) override;
    virtual void do_compute_energy(EnergyInfo& info) override;
    virtual void do_assemble(ContactInfo& info) override;

  private:
    HalfPlane* half_plane = nullptr;
};
}  // namespace uipc::backend::cuda
