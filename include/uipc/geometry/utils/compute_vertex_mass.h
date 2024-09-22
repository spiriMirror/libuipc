#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_vertex_mass(SimplicialComplex& R,
                                                              Float mass_density);
}  // namespace uipc::geometry
