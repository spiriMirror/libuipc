#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Label the vertex color of a simplicial complex by graph coloring algorithm.
 *
 *  The edges of the graph is from the edges of the simplicial complex.
 * - Create a `graph/color` <IndexT> attribute on `vertices` to tell the color of each vertex.
 * - Create a `graph/color_count` <IndexT> attribute on `meta` to tell how many colors are used.
 *
 * @return S<AttributeSlot<IndexT>> The `graph/color` attribute slot.
 */
UIPC_GEOMETRY_API S<AttributeSlot<IndexT>> label_graph_color(SimplicialComplex& sc);
}  // namespace uipc::geometry