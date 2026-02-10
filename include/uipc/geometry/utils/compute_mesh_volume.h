#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Compute the volume of the simplicial complex.
 * 
 * Only tetmesh and closed trimesh are supported.
 * 
 * @param R The simplicial complex.
 * @return The volume of the simplicial complex.
 */
UIPC_GEOMETRY_API Float compute_mesh_volume(SimplicialComplex& R);
}  // namespace uipc::geometry
