#pragma once
#include <contact_system/vertex_half_plane_normal_contact.h>
#include <implicit_geometry/half_plane.h>
namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneNormalContact final : public VertexHalfPlaneNormalContact
{
  public:
    using VertexHalfPlaneNormalContact::VertexHalfPlaneNormalContact;

    virtual void do_build(BuildInfo& info) override;
    virtual void do_compute_energy(EnergyInfo& info) override;
    virtual void do_assemble(ContactInfo& info) override;

  private:
    HalfPlane* half_plane = nullptr;
};
}  // namespace uipc::backend::cuda
