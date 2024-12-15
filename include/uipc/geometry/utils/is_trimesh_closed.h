#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Check if a trimesh is closed. 
 * 
 * Only 2D SimplicialComplex is supported.
 * 
 * @param R the simplicial complex to be checked.
 * @return true if the trimesh is closed, false otherwise.
 */
UIPC_GEOMETRY_API bool is_trimesh_closed(const SimplicialComplex& R);
}  // namespace uipc::geometry
