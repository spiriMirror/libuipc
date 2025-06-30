#include <contact_system/contact_system_feature.h>
#include <contact_system/contact_system_exporter.h>
namespace uipc::backend::cuda
{
ContactSystemFeatureOverrider::ContactSystemFeatureOverrider(ContactSystemExporter* exporter)
{
    UIPC_ASSERT(exporter, "Exporter must not be null");
    m_exporter = *exporter;
}

void ContactSystemFeatureOverrider::get_contact_gradient(geometry::Geometry& vert_grad)
{
    m_exporter->get_contact_gradient(vert_grad);
}

void ContactSystemFeatureOverrider::get_contact_hessian(geometry::Geometry& vert_hess)
{
    m_exporter->get_contact_hessian(vert_hess);
}

void ContactSystemFeatureOverrider::get_contact_primtives(std::string_view prim_type,
                                                          geometry::Geometry& prims)
{
    m_exporter->get_contact_primtives(prim_type, prims);
}

vector<std::string> ContactSystemFeatureOverrider::get_contact_primitive_types() const
{
    return m_exporter->get_contact_primitive_types();
}
}  // namespace uipc::backend::cuda