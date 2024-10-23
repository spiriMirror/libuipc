#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief partition the simplicial complex
 * 
 * create a `mesh_part` <IndexT> attribute on the simplicial complex' vertices
 * 
 * @param sc simplicial complex
 * @param part_max_size the vertex number in each partition <= part_max_size
 * 
 */
void UIPC_GEOMETRY_API mesh_partition(SimplicialComplex& sc, SizeT part_max_size);
}  // namespace uipc::geometry
