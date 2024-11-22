#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry::affine_body
{
/**
 * @brief Compute the dyadic mass of a simplicial complex.
 * 
 * 
 * Integrate the mass density over the simplicial complex to compute the dyadic mass.
 * 
 * 
 * @param[in] sc The simplicial complex.
 * 
 * @param[out] m The total mass.
 * 
 * @param[out] m_x_bar The total mass times the center of mass.
 * 
 * @param[out] m_x_bar_x_bar The total mass times the center of mass times the center of mass transpose.
 */
UIPC_GEOMETRY_API void compute_dyadic_mass(const SimplicialComplex& sc,
                                           Float                    rho,
                                           //tex: $$ \sum \mathbf{m} $$
                                           Float& m,
                                           //tex: $$ \sum \mathbf{m} \bar{\mathbf{x}} $$
                                           Vector3& m_x_bar,
                                           //tex: $$ \sum \mathbf{m} \bar{\mathbf{x}} \cdot \bar{\mathbf{x}}^T$$
                                           Matrix3x3& m_x_bar_x_bar);
}  // namespace uipc::geometry::affine_body
