#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
UIPC_CORE_API [[nodiscard]] SimplicialComplex extract_surface(const SimplicialComplex& src);
}  // namespace uipc::geometry
