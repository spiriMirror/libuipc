#include <uipc/core/contact_system_feature.h>

namespace uipc::core
{
ContactSystemFeature::ContactSystemFeature(S<ContactSystemFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
}

void uipc::core::ContactSystemFeature::contact_gradient(geometry::Geometry& vert_grad)
{
    m_impl->get_contact_gradient(vert_grad);
}

void uipc::core::ContactSystemFeature::contact_hessian(geometry::Geometry& vert_hess)
{
    m_impl->get_contact_hessian(vert_hess);
}

void uipc::core::ContactSystemFeature::contact_primitives(std::string_view prim_type,
                                                          geometry::Geometry& prims)
{
    m_impl->get_contact_primtives(prim_type, prims);
}

vector<std::string> uipc::core::ContactSystemFeature::contact_primitive_types() const
{
    return m_impl->get_contact_primitive_types();
}

std::string_view uipc::core::ContactSystemFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::core