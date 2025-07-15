#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Suggest a proper d_hat for a mesh, create an attribute `d_hat` on meta.
 * 
 * @param R The simplicial complex to compute d_hat for.
 * @param max_d_hat The maximum allowed d_hat, default to infinity.
 */
UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_mesh_d_hat(
    SimplicialComplex& R, Float max_d_hat = std::numeric_limits<Float>::max());
}  // namespace uipc::geometry
