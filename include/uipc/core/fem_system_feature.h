#pragma once
#include <uipc/core/feature.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/constitution/constitution.h>

namespace uipc::core
{
/**
 * @brief Backend hook of the FEMSystemFeature.
 *
 * Implemented by backends that can export finite element data.
 */
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
 * The feature is provided by backends that implement the hook
 * (currently the CUDA backend). Data is recomputed on demand at the
 * backend's current positions, so it matches the state retrieved by
 * `World::retrieve()`.
 *
 * All quantities carry the same `dt^2` time-integration scaling as the
 * solver's incremental potential (consistent with the
 * ContactSystemFeature exports): divide by `dt^2` to obtain the
 * physical values (energy in J, gradient in J/m, Hessian in J/m^2).
 *
 * Ref: https://github.com/spiriMirror/libuipc/issues/344
 */
class UIPC_CORE_API FEMSystemFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "core/fem_system";

    FEMSystemFeature(S<FEMSystemFeatureOverrider> overrider);

    /**
     * @brief Per-element elastic energy of a constitution.
     *
     * Fills `energy` (in place) with one instance per element:
     * - "topo" <Vector4i/Vector3i/Vector2i> element vertex indices
     *   (global FEM vertex ids; absent for particles and extra constitutions)
     * - "energy" <Float> dt^2-scaled elastic energy of the element
     *   (divide by dt^2 for J)
     *
     * Note: some constitutions have a nonzero energy at the rest state by
     * design. E.g. StableNeoHookean (Stiff-GIPC SNK1) evaluates to
     * Psi(I) = mu^2 / (2*lambda_hat) > 0 at the identity deformation, so its
     * exported energies are offset by this constant at rest; subtract it
     * client-side if the offset is unwanted (it is constant, so it does not
     * affect the exported gradient and Hessian).
     *
     * @param uid Constitution UID (see `fem_constitution_uids()`).
     * @param energy Geometry receiving the per-element energies.
     */
    void fem_energy(U64 uid, geometry::Geometry& energy);

    /**
     * @brief Per-element elastic gradient of a constitution.
     *
     * Fills `vert_grad` (in place) with one instance per element-corner:
     * - "i" <IndexT> global FEM vertex index
     * - "grad" <Vector3> dt^2-scaled partial derivative of the elastic
     *   energy w.r.t. the vertex position (divide by dt^2 for J/m). The
     *   elastic force on a vertex is minus the sum of all its "grad"
     *   entries (divided by dt^2).
     *
     * @param uid Constitution UID.
     * @param vert_grad Geometry receiving the gradient doublets.
     */
    void fem_gradient(U64 uid, geometry::Geometry& vert_grad);

    /**
     * @brief Per-element elastic Hessian of a constitution.
     *
     * Fills `vert_hess` (in place) with one instance per element block pair:
     * - "i" <IndexT>, "j" <IndexT> global FEM vertex indices
     * - "hess" <Matrix3x3> dt^2-scaled 3x3 block of the (symmetrized)
     *   elastic Hessian (divide by dt^2 for J/m^2)
     *
     * @param uid Constitution UID.
     * @param vert_hess Geometry receiving the Hessian triplets.
     */
    void fem_hessian(U64 uid, geometry::Geometry& vert_hess);

    /**
     * @brief Per-vertex aggregated elastic gradient of a constitution.
     *
     * Adds a "gradient" <Vector3> vertex attribute to `vert_grad`, holding
     * the dt^2-scaled sum of the per-element gradients acting on each
     * vertex (divide by dt^2 for J/m). The elastic force on the vertex is
     * `-gradient / dt^2` [N].
     *
     * `vert_grad` must be created by
     * `FiniteElementStateAccessorFeature::create_geometry()`, which
     * defines the exported vertex range.
     *
     * @param uid Constitution UID.
     * @param vert_grad State-accessor geometry receiving the vertex gradient.
     */
    void fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad);

    /**
     * @brief UIDs of the constitutions available for export.
     *
     * These are the uids of the finite element constitutions (and extra
     * constitutions, e.g. bending) used by the current scene: the backend
     * only builds constitutions whose uid appears in the scene's contact
     * tabular, so the list is empty before `World::init()` and changes if
     * the scene constitution set changes.
     */
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
