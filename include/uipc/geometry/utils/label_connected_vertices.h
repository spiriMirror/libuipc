#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the connected vertices of a simplicial complex (by edges).
 * 
 * - Create a `region` <IndexT> attribute on `vertices` to tell which region a vertex is belong to.
 * - Create a `region_count` <IndexT> attribute on `meta` to tell how many regions are there.
 * 
 * @return S<AttributeSlot<IndexT>> The `region` attribute slot.
 */
UIPC_GEOMETRY_API S<AttributeSlot<IndexT>> label_connected_vertices(SimplicialComplex& complex);
}  // namespace uipc::geometry
