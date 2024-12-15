#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the regions of a simplicial complex.
 * 
 * - Create a `region` <IndexT> attribute on `edges` to tell which region an edge is belong to.
 * - Create a `region` <IndexT> attribute on `triangles` to tell which region a triangle is belong to. (if exists)
 * - Create a `region` <IndexT> attribute on `tetrahedra` to tell which region a tetrahedron is belong to. (if exists)
 * - Create a `region_count` <IndexT> attribute on `meta` to tell how many regions are there.
 * 
 * @return S<AttributeSlot<IndexT>> The `region` attribute slot.
 */
UIPC_GEOMETRY_API void label_region(SimplicialComplex& complex);
}  // namespace uipc::geometry
