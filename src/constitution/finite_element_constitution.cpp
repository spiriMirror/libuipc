#include <uipc/constitution/finite_element_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_vertex_volume.h>

namespace uipc::constitution
{
void FiniteElementConstitution::apply_to(geometry::SimplicialComplex& sc,
                                         Float mass_density,
                                         Float thickness) const
{
    auto P = sc.meta().find<U64>(builtin::constitution_uid);

    if(!P)
        P = sc.meta().create<U64>(builtin::constitution_uid, 0);
    geometry::view(*P).front() = uid();

    auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
    if(!dof_offset)
        dof_offset = sc.meta().create<IndexT>(builtin::dof_offset, -1);

    auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
    if(!dof_count)
        dof_count = sc.meta().create<IndexT>(builtin::dof_count, 0);

    auto is_fixed = sc.vertices().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.vertices().create<IndexT>(builtin::is_fixed, 0);

    auto is_dynamic = sc.vertices().find<IndexT>(builtin::is_dynamic);
    if(!is_dynamic)
        is_dynamic = sc.vertices().create<IndexT>(builtin::is_dynamic, 1);

    auto attr_thickness = sc.vertices().find<Float>(builtin::thickness);
    if(!attr_thickness)
        attr_thickness = sc.vertices().create<Float>(builtin::thickness, 0.0);

    auto thickness_view = geometry::view(*attr_thickness);
    std::ranges::fill(thickness_view, thickness);

    geometry::compute_vertex_volume(sc);

    auto meta_mass = sc.meta().find<Float>(builtin::mass_density);

    if(!meta_mass)
        meta_mass = sc.meta().create<Float>(builtin::mass_density, mass_density);
    else
        geometry::view(*meta_mass).front() = mass_density;
}
}  // namespace uipc::constitution
