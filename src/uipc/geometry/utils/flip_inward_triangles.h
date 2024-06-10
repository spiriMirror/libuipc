#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_CORE_API [[nodiscard]] SimplicialComplex flip_inward_triangles(const SimplicialComplex& sc);
}
