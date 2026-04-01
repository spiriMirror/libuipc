#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>

namespace uipc::geometry::affine_body
{
// ref: libuipc/scripts/symbol_calculation/rigid_body_to_affine_body.ipynb

// q = [t, a1, a2, a3], where t is translation, A = [a1|a2|a3]^T
//
// M = | m*I_3              I_3 (x) (mc^T)  |
//     | I_3 (x) (mc)       I_3 (x) S       |
//
UIPC_GEOMETRY_API Matrix12x12 build_abd_mass_matrix(Float             m,
                                                    const Vector3&    m_x_bar,
                                                    const Matrix3x3&  m_x_bar_x_bar)
{
    Matrix12x12 M = Matrix12x12::Zero();

    // translation block: M[0:3, 0:3] = m * I_3
    M(0, 0) = m;
    M(1, 1) = m;
    M(2, 2) = m;

    // coupling blocks: M[t_i, A_i*] = mc^T,  M[A_i*, t_i] = mc
    M.block<1, 3>(0, 3)  = m_x_bar.transpose();  // t1 <-> a1
    M.block<3, 1>(3, 0)  = m_x_bar;
    M.block<1, 3>(1, 6)  = m_x_bar.transpose();  // t2 <-> a2
    M.block<3, 1>(6, 1)  = m_x_bar;
    M.block<1, 3>(2, 9)  = m_x_bar.transpose();  // t3 <-> a3
    M.block<3, 1>(9, 2)  = m_x_bar;

    // affine blocks: M[A_i*, A_i*] = S  (three diagonal 3x3 copies)
    M.block<3, 3>(3, 3)  = m_x_bar_x_bar;        // a1-a1
    M.block<3, 3>(6, 6)  = m_x_bar_x_bar;        // a2-a2
    M.block<3, 3>(9, 9)  = m_x_bar_x_bar;        // a3-a3

    return M;
}

UIPC_GEOMETRY_API Matrix12x12 from_rigid_body(
    Float            mass,
    const Vector3&   center_of_mass,
    const Matrix3x3& inertia_cm)
{
    const auto& c = center_of_mass;

    // Step 1: Parallel axis theorem -- shift inertia to origin
    // I^O = I_cm + m * (|c|^2 * I_3 - c * c^T)
    Matrix3x3 I_origin =
        inertia_cm + mass * (c.squaredNorm() * Matrix3x3::Identity() - c * c.transpose());

    // Step 2: Inertia-to-second-moment inversion
    // S = 0.5 * tr(I^O) * I_3 - I^O
    Matrix3x3 S = 0.5 * I_origin.trace() * Matrix3x3::Identity() - I_origin;

    // Step 3: Assemble the 12x12 ABD mass matrix from (m, m*c, S)
    Vector3 m_x_bar = mass * c;

    return build_abd_mass_matrix(mass, m_x_bar, S);
}

UIPC_GEOMETRY_API void to_rigid_body(Float             m,
                                     const Vector3&    m_x_bar,
                                     const Matrix3x3&  m_x_bar_x_bar,
                                     Float&            total_mass,
                                     Vector3&          center_of_mass,
                                     Matrix3x3&        inertia_cm)
{
    total_mass = m;

    if(m <= 0)
    {
        center_of_mass = Vector3::Zero();
        inertia_cm     = Matrix3x3::Zero();
        return;
    }

    center_of_mass = m_x_bar / m;

    // Step 1: Second moment S -> inertia about origin
    //   I^O = tr(S) * I_3 - S
    const Matrix3x3& S        = m_x_bar_x_bar;
    Matrix3x3        I_origin = S.trace() * Matrix3x3::Identity() - S;

    // Step 2: Parallel axis theorem in reverse (origin -> center of mass)
    //   I_cm = I^O - m * (|c|^2 * I_3 - c * c^T)
    Float c2   = center_of_mass.squaredNorm();
    inertia_cm = I_origin
                 - m * (c2 * Matrix3x3::Identity()
                        - center_of_mass * center_of_mass.transpose());
}

UIPC_GEOMETRY_API void to_rigid_body(const Matrix12x12& mass_matrix,
                                     Float&             total_mass,
                                     Vector3&           center_of_mass,
                                     Matrix3x3&         inertia_cm)
{
    Float     m             = mass_matrix(0, 0);
    Vector3   m_x_bar       = mass_matrix.block<3, 1>(3, 0);
    Matrix3x3 m_x_bar_x_bar = mass_matrix.block<3, 3>(3, 3);

    to_rigid_body(m, m_x_bar, m_x_bar_x_bar, total_mass, center_of_mass, inertia_cm);
}
}  // namespace uipc::geometry::affine_body
