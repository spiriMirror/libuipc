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

/**
 * @brief Compute the dyadic mass of a codimensional simplicial complex (shell or rod).
 * 
 * Integrates the mass density over the effective 3D volume of the codim body,
 * accounting for thickness to produce an invertible 12×12 ABD mass matrix.
 * 
 * - dim==2 (shell): direct area integration × 2r, plus normal-direction correction.
 * - dim==1 (rod):   segment integration × π r², plus cross-section correction.
 * 
 * @param[in] sc        The simplicial complex (triangle mesh for shell, edge mesh for rod).
 * @param[in] rho       Volume mass density (kg/m³).
 * @param[in] thickness Thickness radius `r` (half-thickness for shell, cross-section radius for rod).
 * @param[out] m             The total mass.
 * @param[out] m_x_bar       The total mass times the center of mass.
 * @param[out] m_x_bar_x_bar The second dyadic moment (including thickness correction).
 */
UIPC_GEOMETRY_API void compute_dyadic_mass(const SimplicialComplex& sc,
                                           Float                    rho,
                                           Float                    thickness,
                                           Float&                   m,
                                           Vector3&                 m_x_bar,
                                           Matrix3x3& m_x_bar_x_bar);
}  // namespace uipc::geometry::affine_body
