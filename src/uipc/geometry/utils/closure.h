#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Generate the closure of a simplicial complex, who only have the top dimension simplices.
 * 
 * E.g. 
 * 1) the input 3D simplicial complex can only have tetrahedrons (no triangles, edges).
 * 2) the input 2D simplicial complex can only have triangles (no edges).
 * 3) the input 1D simplicial complex can only have edges.
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex pure_closure(const SimplicialComplex& complex);
}  // namespace uipc::geometry
