#include <uipc/constitution/fem_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>

namespace uipc::constitution
{
constitution::ConstitutionType FiniteElementConstitution::get_type() const noexcept
{
    return constitution::ConstitutionType::FiniteElement;
}

void FiniteElementConstitution::apply_to(geometry::SimplicialComplex& sc,
                                         Float mass_density,
                                         Float thickness) const
{
    auto P = sc.meta().find<U64>(builtin::constitution_uid);

    if(!P)
        P = sc.meta().create<U64>(builtin::constitution_uid, uid());
    else
        geometry::view(*P).front() = uid();

    auto is_fixed = sc.vertices().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.vertices().create<IndexT>(builtin::is_fixed, 0);

    geometry::compute_vertex_mass(sc, mass_density);

    auto attr_thickness = sc.vertices().find<Float>(builtin::thickness);
    if(!attr_thickness)
        attr_thickness = sc.vertices().create<Float>(builtin::thickness, thickness);
    else
    {
        auto thickness_view = geometry::view(*attr_thickness);
        std::ranges::fill(thickness_view, thickness);
    }
}
}  // namespace uipc::constitution
