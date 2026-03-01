#include <uipc/constitution/affine_body_rod.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_mesh_length.h>

namespace uipc::constitution
{
AffineBodyRod::AffineBodyRod(const Json& config) noexcept
    : AffineBodyConstitution(config)
{
}

void AffineBodyRod::apply_to(geometry::SimplicialComplex& sc,
                             Float                        kappa,
                             Float                        mass_density,
                             Float                        thickness) const
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyRod requires a 1D simplicial complex (edge mesh), got dim={}.",
                sc.dim());

    // Compute effective volume = total length * pi * r^2
    Float volume = geometry::compute_rod_volume(sc, thickness);

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



