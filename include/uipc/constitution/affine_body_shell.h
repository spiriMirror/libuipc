#pragma once
#include <uipc/constitution/affine_body_constitution.h>

namespace uipc::constitution
{
/**
 * @brief Codimensional 2D (shell) affine body constitution.
 * 
 * Convenience subclass of AffineBodyConstitution for open triangle meshes
 * treated as thin shells with a given thickness.
 * 
 * Uses the same UID as AffineBodyConstitution (OrthoPotential = 1 by default).
 * Sets `builtin::is_codim = 1` on the geometry meta so the backend uses
 * the codim dyadic mass computation path.
 */
class UIPC_CONSTITUTION_API AffineBodyShell : public AffineBodyConstitution
{
  public:
    AffineBodyShell(const Json& config = default_config()) noexcept;

    /**
     * @brief Apply the shell constitution to a 2D simplicial complex.
     * 
     * @param sc           The triangle mesh (dim == 2).
     * @param kappa        Stiffness parameter for shape energy.
     * @param mass_density Volume mass density (kg/m³).
     * @param thickness    Shell thickness radius `r` (effective slab thickness = 2r).
     */
    void apply_to(geometry::SimplicialComplex& sc,
                  Float                        kappa,
                  Float                        mass_density = 1e3,
                  Float                        thickness    = 0.01) const;
};
}  // namespace uipc::constitution



