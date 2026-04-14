#include <uipc/constitution/affine_body_rod.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_mesh_length.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>

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
    UIPC_ASSERT_THROW(sc.dim() == 1,
                "AffineBodyRod requires a 1D simplicial complex (edge mesh), got dim={}.",
                sc.dim());

    Float volume = geometry::compute_rod_volume(sc, thickness);

    Float m;
    Vector3 m_x_bar;
    Matrix3x3 m_x_bar_x_bar;
    geometry::affine_body::compute_dyadic_mass(sc, mass_density, thickness, m, m_x_bar, m_x_bar_x_bar);
    create_abd_attributes(sc, kappa, mass_density, volume, m, m_x_bar, m_x_bar_x_bar);

    auto is_codim = sc.meta().find<IndexT>(builtin::is_codim);
    if(!is_codim)
        is_codim = sc.meta().create<IndexT>(builtin::is_codim, 1);
    else
        geometry::view(*is_codim).front() = 1;

    auto attr_thickness = sc.vertices().find<Float>(builtin::thickness);
    if(!attr_thickness)
        attr_thickness = sc.vertices().create<Float>(builtin::thickness, 0.0);
    auto thickness_view = geometry::view(*attr_thickness);
    std::ranges::fill(thickness_view, thickness);
}
}  // namespace uipc::constitution



