#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the surface of a simplicial complex.
 * 
 * 1) label 'is_surf':<IndexT> on vertices/edges/triangles/tetrahedra
 * 2) set  'parent_id':<IndexT> on triangles, indicating the parent tetrahedron
 * 
 * @param R the simplicial complex to be labeled.
 */
UIPC_GEOMETRY_API void label_surface(SimplicialComplex& R);
}  // namespace uipc::geometry
