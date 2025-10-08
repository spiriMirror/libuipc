#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Construct a point cloud inside a volume represented by a simplicial complex.
 * 
 * @param sc The input simplicial complex representing a volume (closed manifold).
 * @param resolution The resolution of the point cloud.
 * 
 * @return SimplicialComplex A point cloud represented as a simplicial complex with only vertices.
 */
UIPC_GEOMETRY_API SimplicialComplex points_from_volume(const SimplicialComplex& sc,
                                                       Float resolution = 0.01);
}  // namespace uipc::geometry
