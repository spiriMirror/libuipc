#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_CORE_API [[nodiscard]] SimplicialComplex label_triangle_orient(const SimplicialComplex& sc);
}
