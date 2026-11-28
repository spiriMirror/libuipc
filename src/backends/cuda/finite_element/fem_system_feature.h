#pragma once
#include <sim_system.h>
#include <uipc/core/fem_system_feature.h>

namespace uipc::backend::cuda
{
class FEMExporterManager;

class FEMSystemFeatureOverrider final : public core::FEMSystemFeatureOverrider
{
  public:
    FEMSystemFeatureOverrider(FEMExporterManager* exporter_manager);

  private:
    void get_fem_energy(U64 uid, geometry::Geometry& prim_energy) override;
    void get_fem_gradient(U64 uid, geometry::Geometry& prim_grad) override;
    void get_fem_hessian(U64 uid, geometry::Geometry& prim_hess) override;
    void get_fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad) override;
    vector<U64> get_fem_constitution_uids() const override;

    SimSystemSlot<FEMExporterManager> m_exporter_manager;
};
}  // namespace uipc::backend::cuda
