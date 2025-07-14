#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_proper_d_hat(SimplicialComplex& R);
}
