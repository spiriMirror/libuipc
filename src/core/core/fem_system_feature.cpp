#include <uipc/core/fem_system_feature.h>

namespace uipc::core
{
FEMSystemFeature::FEMSystemFeature(S<FEMSystemFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
}

void FEMSystemFeature::fem_energy(U64 uid, geometry::Geometry& prims)
{
    m_impl->get_fem_energy(uid, prims);
}

void FEMSystemFeature::fem_gradient(U64 uid, geometry::Geometry& vert_grad)
{
    m_impl->get_fem_gradient(uid, vert_grad);
}

void FEMSystemFeature::fem_hessian(U64 uid, geometry::Geometry& vert_hess)
{
    m_impl->get_fem_hessian(uid, vert_hess);
}

void FEMSystemFeature::fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad)
{
    m_impl->get_fem_vertex_gradient(uid, vert_grad);
}

vector<U64> FEMSystemFeature::fem_constitution_uids() const
{
    return m_impl->get_fem_constitution_uids();
}

void FEMSystemFeature::fem_energy(const constitution::IConstitution& c,
                                  geometry::Geometry&                prims)
{
    m_impl->get_fem_energy(c.uid(), prims);
}

void FEMSystemFeature::fem_gradient(const constitution::IConstitution& c,
                                    geometry::Geometry& vert_grad)
{
    m_impl->get_fem_gradient(c.uid(), vert_grad);
}

void FEMSystemFeature::fem_hessian(const constitution::IConstitution& c,
                                   geometry::Geometry&                vert_hess)
{
    m_impl->get_fem_hessian(c.uid(), vert_hess);
}

void FEMSystemFeature::fem_vertex_gradient(const constitution::IConstitution& c,
                                           geometry::SimplicialComplex& vert_grad)
{
    m_impl->get_fem_vertex_gradient(c.uid(), vert_grad);
}

std::string_view FEMSystemFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::core
