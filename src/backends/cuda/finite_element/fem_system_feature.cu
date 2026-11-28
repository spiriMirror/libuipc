#include <finite_element/fem_system_feature.h>
#include <finite_element/fem_exporter_manager.h>

namespace uipc::backend::cuda
{
FEMSystemFeatureOverrider::FEMSystemFeatureOverrider(FEMExporterManager* exporter_manager)
{
    UIPC_ASSERT(exporter_manager, "Exporter manager must not be null");
    m_exporter_manager = *exporter_manager;
}

void FEMSystemFeatureOverrider::get_fem_energy(U64 uid, geometry::Geometry& prim_energy)
{
    m_exporter_manager->get_fem_energy(uid, prim_energy);
}

void FEMSystemFeatureOverrider::get_fem_gradient(U64 uid, geometry::Geometry& prim_grad)
{
    m_exporter_manager->get_fem_gradient(uid, prim_grad);
}

void FEMSystemFeatureOverrider::get_fem_hessian(U64 uid, geometry::Geometry& prim_hess)
{
    m_exporter_manager->get_fem_hessian(uid, prim_hess);
}

void FEMSystemFeatureOverrider::get_fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad)
{
    m_exporter_manager->get_fem_vertex_gradient(uid, vert_grad);
}

vector<U64> FEMSystemFeatureOverrider::get_fem_constitution_uids() const
{
    return m_exporter_manager->get_fem_constitution_uids();
}
}  // namespace uipc::backend::cuda
