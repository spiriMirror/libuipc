#include <contact_system/contact_system_feature.h>
#include <contact_system/contact_exporter_manager.h>

namespace uipc::backend::cuda
{
ContactSystemFeatureOverrider::ContactSystemFeatureOverrider(ContactExporterManager* manager)
{
    UIPC_ASSERT(manager, "Exporter must not be null");
    m_manager = *manager;
}

vector<std::string> ContactSystemFeatureOverrider::get_contact_primitive_types() const
{
    return m_manager->get_contact_primitive_types();
}

void ContactSystemFeatureOverrider::get_contact_gradient(std::string_view prim_type,
                                                         geometry::Geometry& vert_grad)
{
    m_manager->get_contact_gradient(prim_type, vert_grad);
}

void ContactSystemFeatureOverrider::get_contact_hessian(std::string_view prim_type,
                                                        geometry::Geometry& vert_hess)
{
    m_manager->get_contact_hessian(prim_type, vert_hess);
}
void ContactSystemFeatureOverrider::get_contact_energy(std::string_view prim_type,
                                                       geometry::Geometry& prims)
{
    m_manager->get_contact_energy(prim_type, prims);
}
}  // namespace uipc::backend::cuda