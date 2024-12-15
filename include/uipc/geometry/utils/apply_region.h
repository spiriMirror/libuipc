#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Take apart the simplicial complex by regions.
 * 
 * @return vector<SimplicialComplex> The simplicial complexes by regions.
 */
UIPC_GEOMETRY_API vector<SimplicialComplex> apply_region(const SimplicialComplex& complex);
}  // namespace uipc::geometry
