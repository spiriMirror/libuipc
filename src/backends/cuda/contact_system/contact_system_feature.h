#pragma once
#include <sim_system.h>
#include <uipc/core/contact_system_feature.h>

namespace uipc::backend::cuda
{
class ContactSystemExporter;

class ContactSystemFeatureOverrider final : public core::ContactSystemFeatureOverrider
{
  public:
    ContactSystemFeatureOverrider(ContactSystemExporter* contact_system);

  protected:
    void get_contact_gradient(geometry::Geometry& vert_grad) override;

    void get_contact_hessian(geometry::Geometry& vert_hess) override;

    void get_contact_primtives(std::string_view prim_type, geometry::Geometry& prims) override;

    vector<std::string> get_contact_primitive_types() const override;

  private:
    SimSystemSlot<ContactSystemExporter> m_exporter;
};
}  // namespace uipc::backend::cuda