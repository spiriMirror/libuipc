#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Flip the orientation of the triangles in the simplicial complex.
 * 
 * The input simplicial complex should have the attribute slot 'orient':<Index> for each triangle.
 * After the operation, the orientation of the triangles will be flipped, and the `orient` attribute will be updated.
 * 
 * @return SimplicialComplex the simplicial complex with the orientation of the triangles flipped.
 */
UIPC_GEOMETRY_API [[nodiscard]] SimplicialComplex flip_inward_triangles(const SimplicialComplex& sc);
}  // namespace uipc::geometry
