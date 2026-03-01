#include <uipc/constitution/affine_body_shell.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_mesh_area.h>

namespace uipc::constitution
{
AffineBodyShell::AffineBodyShell(const Json& config) noexcept
    : AffineBodyConstitution(config)
{
}

void AffineBodyShell::apply_to(geometry::SimplicialComplex& sc,
                               Float                        kappa,
                               Float                        mass_density,
                               Float                        thickness) const
{
    UIPC_ASSERT(sc.dim() == 2,
                "AffineBodyShell requires a 2D simplicial complex (triangle mesh), got dim={}.",
                sc.dim());

    // Compute effective volume = total area * 2r
    Float volume = geometry::compute_mesh_area(sc, thickness);

    // Set common ABD attributes (constitution_uid, kappa, volume, mass_density, etc.)
    setup_abd_attributes(sc, kappa, mass_density, volume);

    // Mark as codimensional so the backend uses the codim dyadic mass path
    auto is_codim = sc.meta().find<IndexT>(builtin::is_codim);
    if(!is_codim)
        is_codim = sc.meta().create<IndexT>(builtin::is_codim, 1);
    else
        geometry::view(*is_codim).front() = 1;

    // Set thickness on vertices (same convention as FEM)
    auto attr_thickness = sc.vertices().find<Float>(builtin::thickness);
    if(!attr_thickness)
        attr_thickness = sc.vertices().create<Float>(builtin::thickness, thickness);
    auto thickness_view = geometry::view(*attr_thickness);
    std::ranges::fill(thickness_view, thickness);
}
}  // namespace uipc::constitution



