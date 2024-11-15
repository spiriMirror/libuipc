#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Tetrahedralize a 2D simplicial complex (trimesh).
 * 
 * @return SimplicialComplex The simplicial complexes by regions.
 */
UIPC_GEOMETRY_API SimplicialComplex tetrahedralize(const SimplicialComplex& sc,
                                                   const Json& options = Json::object());
}  // namespace uipc::geometry
