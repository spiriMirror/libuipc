#pragma once
#include <sim_system.h>
#include <uipc/core/contact_system_feature.h>

namespace uipc::backend::cuda
{
class ContactExporterManager;

class ContactSystemFeatureOverrider final : public core::ContactSystemFeatureOverrider
{
  public:
    ContactSystemFeatureOverrider(ContactExporterManager* contact_system);

  private:
    vector<std::string> get_contact_primitive_types() const override;

    void do_compute_contact() override;

    void get_contact_gradient(geometry::Geometry& vert_grad) override;
    void get_contact_hessian(geometry::Geometry& vert_hess) override;

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override;
    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override;
    void get_contact_energy(std::string_view prim_type, geometry::Geometry& prims) override;

    SimSystemSlot<ContactExporterManager> m_manager;
};
}  // namespace uipc::backend::cuda