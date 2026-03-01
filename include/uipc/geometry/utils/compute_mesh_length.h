#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Compute the effective volume of an edge mesh treated as a thin rod.
 * 
 * Sums `length_i * pi * r^2` over all edges, where `length_i = |e1|`
 * and `r` is the thickness (cross-section radius).
 * 
 * Requires `sc.dim() == 1`.
 * 
 * @param sc The simplicial complex (edge mesh).
 * @param thickness The rod cross-section radius `r`.
 * @return The effective volume (= total length * pi * r^2).
 */
UIPC_GEOMETRY_API Float compute_rod_volume(const SimplicialComplex& sc,
                                           Float                    thickness);
}  // namespace uipc::geometry



