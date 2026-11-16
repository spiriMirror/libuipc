#pragma once
#include <uipc/core/feature.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/constitution/constitution.h>

namespace uipc::core
{
class UIPC_CORE_API FEMSystemFeatureOverrider
{
  public:
    virtual ~FEMSystemFeatureOverrider() = default;

    virtual void get_fem_energy(U64 uid, geometry::Geometry& prim_energy) = 0;

    virtual void get_fem_gradient(U64 uid, geometry::Geometry& prim_grad) = 0;

    virtual void get_fem_hessian(U64 uid, geometry::Geometry& prim_hess) = 0;

    virtual void get_fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad) = 0;

    virtual vector<U64> get_fem_constitution_uids() const = 0;
};

/**
 * @brief Feature to export finite element (FEM) energy, gradient and Hessian data.
 *
 * All quantities carry the same `dt^2` time-integration scaling as the solver's
 * incremental potential: divide by `dt^2` to obtain the physical values.
 */
class UIPC_CORE_API FEMSystemFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "core/fem_system";

    FEMSystemFeature(S<FEMSystemFeatureOverrider> overrider);

    void fem_energy(U64 uid, geometry::Geometry& energy);
    void fem_gradient(U64 uid, geometry::Geometry& vert_grad);
    void fem_hessian(U64 uid, geometry::Geometry& vert_hess);
    void fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad);
    vector<U64> fem_constitution_uids() const;

    void fem_energy(const constitution::IConstitution& c, geometry::Geometry& energy);
    void fem_gradient(const constitution::IConstitution& c, geometry::Geometry& vert_grad);
    void fem_hessian(const constitution::IConstitution& c, geometry::Geometry& vert_hess);
    void fem_vertex_gradient(const constitution::IConstitution& c,
                             geometry::SimplicialComplex&       vert_grad);

  private:
    virtual std::string_view     get_name() const override;
    S<FEMSystemFeatureOverrider> m_impl;
};
}  // namespace uipc::core
