#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_vertex_volume(SimplicialComplex& R);
}  // namespace uipc::geometry
