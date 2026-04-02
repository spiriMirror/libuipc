#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyConstitution : public IConstitution
{
    using Base = IConstitution;

  public:
    AffineBodyConstitution(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density = 1e3) const;

    /**
     * @brief Apply ABD constitution with explicit mass matrix and volume override.
     *
     * The 12x12 mass matrix is decomposed into (m, m_x_bar, m_x_bar_x_bar)
     * and stored as meta attributes so the backend uses them directly
     * instead of computing from mesh geometry.
     */
    void apply_to(geometry::SimplicialComplex& sc,
                  Float                        kappa,
                  const Matrix12x12&           mass,
                  Float                        volume) const;

    /**
     * @brief Create a 1-vertex proxy affine body from rigid body mass properties.
     *
     * The returned SimplicialComplex has a single vertex at the origin and
     * carries all ABD meta attributes.  It participates in dynamics (gravity,
     * shape energy) but has no collision geometry.
     */
    geometry::SimplicialComplex create_proxy(Float              mass,
                                             const Vector3&     mass_center,
                                             const Matrix3x3&   inertia,
                                             Float              volume) const;

    /**
     * @brief Create a 1-vertex proxy affine body from a precomputed 12x12 ABD
     *        mass matrix.
     */
    geometry::SimplicialComplex create_proxy(const Matrix12x12& abd_mass,
                                             Float              volume) const;

    static Json default_config() noexcept;

  protected:
    virtual U64 get_uid() const noexcept override;

    /**
     * @brief Common ABD attribute setup.
     * 
     * Sets constitution_uid, transforms, dof_offset, dof_count, is_fixed,
     * is_dynamic, external_kinetic, velocity, self_collision, kappa,
     * volume, mass_density, abd_mass/abd_mass_x_bar/abd_mass_x_bar_x_bar,
     * and user-facing mass/mass_center/inertia on sc.meta().
     * 
     * Subclasses (AffineBodyShell, AffineBodyRod) call this with their
     * own pre-computed volume and dyadic mass.
     */
    void create_abd_attributes(geometry::SimplicialComplex& sc,
                               Float                        kappa,
                               Float                        mass_density,
                               Float                        volume,
                               Float                        m,
                               const Vector3&               m_x_bar,
                               const Matrix3x3&             m_x_bar_x_bar) const;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
