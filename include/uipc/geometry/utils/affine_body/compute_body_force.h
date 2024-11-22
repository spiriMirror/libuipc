#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry::affine_body
{
/**
 * @brief Compute the body force of an affine body.
 * 
 * @param sc The simplicial complex.
 * @param body_force_density The body force density in N/m^3.
 * @param body_force The body force
 */
UIPC_GEOMETRY_API Vector12 compute_body_force(const SimplicialComplex& sc,
                                              const Vector3& body_force_density);
}  // namespace uipc::geometry::affine_body
