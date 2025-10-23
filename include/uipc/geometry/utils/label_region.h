#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the regions of a simplicial complex.
 *
 * - Create a `region` <IndexT> attribute on `vertices` to tell which region a vertex belongs to.
 * - Create a `region` <IndexT> attribute on `edges` to tell which region an edge belongs to.
 * - Create a `region` <IndexT> attribute on `triangles`  (if exists) to tell which region a triangle belongs to.
 * - Create a `region` <IndexT> attribute on `tetrahedra`  (if exists) to tell which region a tetrahedron belongs to.
 * - Create a `region_count` <IndexT> attribute on `meta` to tell how many regions are there.
 *
 */
UIPC_GEOMETRY_API void label_region(SimplicialComplex& complex);
}  // namespace uipc::geometry
