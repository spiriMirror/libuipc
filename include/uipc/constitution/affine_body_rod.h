#pragma once
#include <uipc/constitution/affine_body_constitution.h>

namespace uipc::constitution
{
/**
 * @brief Codimensional 1D (rod) affine body constitution.
 * 
 * Convenience subclass of AffineBodyConstitution for edge meshes (polylines)
 * treated as thin rods with a circular cross-section of a given radius.
 * 
 * Uses the same UID as AffineBodyConstitution (OrthoPotential = 1 by default).
 * Sets `builtin::is_codim = 1` on the geometry meta so the backend uses
 * the codim dyadic mass computation path.
 */
class UIPC_CONSTITUTION_API AffineBodyRod : public AffineBodyConstitution
{
  public:
    AffineBodyRod(const Json& config = default_config()) noexcept;

    /**
     * @brief Apply the rod constitution to a 1D simplicial complex.
     * 
     * @param sc           The edge mesh (dim == 1).
     * @param kappa        Stiffness parameter for shape energy.
     * @param mass_density Volume mass density (kg/m³).
     * @param thickness    Rod cross-section radius `r` (cross-section area = π r²).
     */
    void apply_to(geometry::SimplicialComplex& sc,
                  Float                        kappa,
                  Float                        mass_density = 1e3,
                  Float                        thickness    = 0.01) const;
};
}  // namespace uipc::constitution



