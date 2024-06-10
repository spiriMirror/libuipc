#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the surface of a simplicial complex.
 * 
 * 1) label 'is_surf':<IndexT> on vertices/edges/triangles/tetrahedra
 * 2) set  'parent_id':<IndexT> on triangles, indicating the parent tetrahedron
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex label_surface(const SimplicialComplex& sc);
}  // namespace uipc::geometry
