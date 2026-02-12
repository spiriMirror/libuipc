#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label open edges in a trimesh.
 * 
 * An edge is considered open if it is shared by exactly 1 triangle.
 * An edge shared by 2 triangles is a normal closed edge.
 * An edge shared by >=3 triangles is invalid and will throw an exception.
 * 
 * - Create an `is_open` <IndexT> attribute on `edges` to mark which edges are open (1 = open, 0 = closed).
 * 
 * Only 2D SimplicialComplex is supported.
 * 
 * @param R the simplicial complex to be checked.
 * @return S<AttributeSlot<IndexT>> The `is_open` attribute slot.
 */
UIPC_GEOMETRY_API S<AttributeSlot<IndexT>> label_open_edge(SimplicialComplex& R);
}  // namespace uipc::geometry

