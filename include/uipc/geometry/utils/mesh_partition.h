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
 * @param part_max_size the vertex number in each partition <= part_max_size.
 * @throws uipc::Exception if part_max_size is zero or the graph exceeds the
 *         32-bit index capacity of the embedded METIS implementation.
 *        NOTE: the CUDA MAS preconditioner hard-codes BANKSIZE=16, so 16 is
 *        the only value it accepts. End users should not call this to enable
 *        MAS; set scene config `linear_system/fem_preconditioner = "mas"`
 *        instead, which auto-partitions every FEM geometry internally with
 *        the fixed size. This function remains for custom C++ partitioning
 *        (a pre-existing mesh_part attribute is respected by MAS as-is).
 * 
 */
void UIPC_GEOMETRY_API mesh_partition(SimplicialComplex& sc, SizeT part_max_size);
}  // namespace uipc::geometry
