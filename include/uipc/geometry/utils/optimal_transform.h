#pragma once
#include "uipc/common/dllexport.h"
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Find out the optimal transformation matrix that maps the source points to the destination points.
 * 
 * @param S The source points.
 * @param D The destination points.
 * @return 
 */
UIPC_GEOMETRY_API Matrix4x4 optimal_transform(span<const Vector3> S, span<const Vector3> D);

/**
 * @brief Find out the optimal transformation matrix that maps the source simplicial complex to the destination simplicial complex.
 * 
 * @param S The source simplicial complex.
 * @param D The destination simplicial complex.
 * @return 
 */
UIPC_GEOMETRY_API Matrix4x4 optimal_transform(const SimplicialComplex& S, const SimplicialComplex& D);
}  // namespace uipc::geometry
