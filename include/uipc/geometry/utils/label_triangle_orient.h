#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the orientation of the triangles in the simplicial complex.
 * 
 * Set 'orient':<Index> for each triangle in the simplicial complex.
 * 1) orient=1 means the triangle is oriented outward the tetrahedron.
 * 2) orient=0 means the orientation is undetermined.
 * 3) orient=-1 means the triangle is oriented inward the tetrahedron.
 * 
 * @return P<AttributeSlot<IndexT>> the attribute slot of the triangle orientation.
 */
UIPC_GEOMETRY_API S<AttributeSlot<IndexT>> label_triangle_orient(SimplicialComplex& sc);
}  // namespace uipc::geometry
