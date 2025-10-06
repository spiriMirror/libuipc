#include <uipc/core/contact_system_feature.h>

namespace uipc::core
{
ContactSystemFeature::ContactSystemFeature(S<ContactSystemFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
}

void ContactSystemFeature::contact_energy(std::string_view prim_type, geometry::Geometry& prims)
{
    m_impl->get_contact_energy(prim_type, prims);
}

void ContactSystemFeature::contact_gradient(std::string_view    prim_type,
                                            geometry::Geometry& vert_grad)
{
    m_impl->get_contact_gradient(prim_type, vert_grad);
}

void ContactSystemFeature::contact_hessian(std::string_view    prim_type,
                                           geometry::Geometry& vert_hess)
{
    m_impl->get_contact_hessian(prim_type, vert_hess);
}

void ContactSystemFeature::contact_energy(const constitution::IConstitution& c,
                                          geometry::Geometry& prims)
{
    auto uid_str = fmt::format("#{}", c.uid());
    m_impl->get_contact_energy(uid_str, prims);
}

void ContactSystemFeature::contact_gradient(const constitution::IConstitution& c,
                                            geometry::Geometry& vert_grad)
{
    auto uid_str = fmt::format("#{}", c.uid());
    m_impl->get_contact_gradient(uid_str, vert_grad);
}

void ContactSystemFeature::contact_hessian(const constitution::IConstitution& c,
                                           geometry::Geometry& vert_hess)
{
    auto uid_str = fmt::format("#{}", c.uid());
    m_impl->get_contact_hessian(uid_str, vert_hess);
}

vector<std::string> ContactSystemFeature::contact_primitive_types() const
{
    return m_impl->get_contact_primitive_types();
}

std::string_view ContactSystemFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::core