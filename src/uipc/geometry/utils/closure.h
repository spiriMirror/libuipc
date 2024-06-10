#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Generate the closure from a collection of facet simplices, who only have the top dimension simplices.
 * 
 * E.g. 
 * 1) the input 3D simplicial complex (tetmesh) can only have tetrahedrons (no triangles, edges).
 * 2) the input 2D simplicial complex (trimesh) can only have triangles (no edges).
 * 3) the input 1D simplicial complex (linemesh) can only have edges.
 * 4) the input 0D simplicial complex (pointcloud) can only have vertices.
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex facet_closure(const SimplicialComplex& complex);
}  // namespace uipc::geometry
