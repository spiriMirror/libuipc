#include <uipc/constitution/fem_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>

namespace uipc::constitution
{
constitution::ConstitutionType FiniteElementConstitution::get_type() const noexcept
{
    return constitution::ConstitutionType::FiniteElement;
}

void FiniteElementConstitution::apply_to(geometry::SimplicialComplex& sc, Float mass_density) const
{
    Base::apply_to(sc);

    auto is_fixed = sc.vertices().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.vertices().create<IndexT>(builtin::is_fixed, 0);

    geometry::compute_vertex_mass(sc, mass_density);
}
}  // namespace uipc::constitution
