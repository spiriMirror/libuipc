#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class AffineBodyConstitution;

class UIPC_CONSTITUTION_API AffineBodyMaterial
{
  public:
    void apply_to(geometry::SimplicialComplex& sc) const;

  private:
    friend class AffineBodyConstitution;
    AffineBodyMaterial(const AffineBodyConstitution&, Float kappa, Float mass_density = 1e3) noexcept;

    const AffineBodyConstitution& m_constitution;
    Float                         m_kappa;
    Float                         m_mass_density;
};

class UIPC_CONSTITUTION_API AffineBodyConstitution : public IConstitution
{
    using Base = IConstitution;

  public:
    AffineBodyConstitution(const Json& config = default_config()) noexcept;
    AffineBodyMaterial create_material(Float kappa) const noexcept;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density = 1e3) const;

    static Json default_config() noexcept;

  protected:
    virtual U64 get_uid() const noexcept override;

    /**
     * @brief Common ABD attribute setup without volume computation.
     * 
     * Sets constitution_uid, transforms, dof_offset, dof_count, is_fixed,
     * is_dynamic, external_kinetic, velocity, self_collision, kappa,
     * volume, and mass_density attributes.
     * 
     * Subclasses (AffineBodyShell, AffineBodyRod) call this with their
     * own pre-computed volume instead of compute_mesh_volume().
     */
    void setup_abd_attributes(geometry::SimplicialComplex& sc,
                              Float                        kappa,
                              Float                        mass_density,
                              Float                        volume) const;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
