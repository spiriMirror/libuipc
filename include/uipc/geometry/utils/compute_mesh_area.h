#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Compute the effective volume of a triangle mesh treated as a thin shell.
 * 
 * Sums `area_i * 2r` over all triangles, where `area_i = |e1 x e2| / 2`
 * and `r` is the thickness (radius / half-thickness).
 * 
 * Works on open (non-closed) meshes. Requires `sc.dim() == 2`.
 * 
 * @param sc The simplicial complex (triangle mesh).
 * @param thickness The shell thickness radius `r`.
 * @return The effective volume (= total area * 2r).
 */
UIPC_GEOMETRY_API Float compute_shell_volume(const SimplicialComplex& sc,
                                             Float                    thickness);
}  // namespace uipc::geometry



