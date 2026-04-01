#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>

namespace uipc::geometry::affine_body
{
/**
 * @brief Build the 12x12 ABD mass matrix from rigid body quantities.
 *
 * Given the standard rigid body properties (mass, center of mass, inertia
 * tensor about the center of mass), this function constructs the equivalent
 * affine body dynamics mass matrix.
 *
 * Recipe (see scripts/symbol_calculation/rigid_body_to_affine_body.ipynb):
 *   1. Parallel axis theorem:  I^O = I_cm + m * (|c|^2 I_3 - c c^T)
 *   2. Inertia-to-second-moment inversion:  S = 0.5 * tr(I^O) * I_3 - I^O
 *   3. Assemble the 12x12 block matrix from (m, m*c, S).
 *
 * @param mass           Total mass of the rigid body.
 * @param center_of_mass Center of mass in the reference configuration.
 * @param inertia_cm     3x3 inertia tensor about the center of mass.
 * @return The 12x12 ABD mass matrix.
 */
UIPC_GEOMETRY_API Matrix12x12 from_rigid_body(
    Float            mass,
    const Vector3&   center_of_mass,
    const Matrix3x3& inertia_cm);

/**
 * @brief Build the 12x12 ABD mass matrix from dyadic mass components.
 *
 * Assembles the block-structured mass matrix:
 *   M = | m*I_3            I_3 (x) (mc^T) |
 *       | I_3 (x) (mc)     I_3 (x) S      |
 *
 * @param m             Total mass.
 * @param m_x_bar       First moment of mass (m * center_of_mass).
 * @param m_x_bar_x_bar Second moment of mass tensor S.
 * @return The 12x12 ABD mass matrix.
 */
UIPC_GEOMETRY_API Matrix12x12 build_abd_mass_matrix(
    Float             m,
    const Vector3&    m_x_bar,
    const Matrix3x3&  m_x_bar_x_bar);

/**
 * @brief Extract rigid body properties from dyadic mass components.
 *
 * Inverse of `from_rigid_body`: given the ABD dyadic mass quantities,
 * recovers the total mass, center of mass, and inertia tensor about
 * the center of mass.
 *
 * @param m               Total mass (= sum of element masses).
 * @param m_x_bar         First moment of mass (m * center_of_mass).
 * @param m_x_bar_x_bar   Second moment of mass tensor S.
 * @param[out] total_mass      Total mass (same as m).
 * @param[out] center_of_mass  Center of mass in the reference frame.
 * @param[out] inertia_cm      3x3 inertia tensor about the center of mass.
 */
UIPC_GEOMETRY_API void to_rigid_body(Float             m,
                                     const Vector3&    m_x_bar,
                                     const Matrix3x3&  m_x_bar_x_bar,
                                     Float&            total_mass,
                                     Vector3&          center_of_mass,
                                     Matrix3x3&        inertia_cm);

/**
 * @brief Extract rigid body properties from a 12x12 ABD mass matrix.
 *
 * Convenience overload that decomposes the mass matrix into (m, m_x_bar,
 * m_x_bar_x_bar) and then calls the dyadic-mass version of `to_rigid_body`.
 *
 * @param mass_matrix     The 12x12 ABD mass matrix.
 * @param[out] total_mass      Total mass.
 * @param[out] center_of_mass  Center of mass in the reference frame.
 * @param[out] inertia_cm      3x3 inertia tensor about the center of mass.
 */
UIPC_GEOMETRY_API void to_rigid_body(const Matrix12x12& mass_matrix,
                                     Float&             total_mass,
                                     Vector3&           center_of_mass,
                                     Matrix3x3&         inertia_cm);
}  // namespace uipc::geometry::affine_body
