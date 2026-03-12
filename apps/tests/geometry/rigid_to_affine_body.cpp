/**
 * Unit tests for rigid body -> affine body mass matrix conversion.
 *
 * Test coverage:
 *   1. Cube consistency: affine_body_from_rigid_body matches compute_dyadic_mass
 *   2. Diagonal inertia special case: verifies the closed-form for S
 *   3. Roundtrip: decompose a 12x12 matrix and reassemble
 */

#include <app/app.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>

using namespace uipc;
using namespace uipc::geometry;
using Catch::Approx;

using affine_body::build_abd_mass_matrix;

// ---------------------------------------------------------------------------
// Helper: check that the 12x12 mass matrix is symmetric, positive-definite,
// and invertible.
// ---------------------------------------------------------------------------
static void check_spd_invertible(const Matrix12x12& M)
{
    REQUIRE(M.isApprox(M.transpose(), 1e-9));

    Eigen::LLT<Matrix12x12> llt(M);
    REQUIRE(llt.info() == Eigen::Success);

    REQUIRE(M.fullPivLu().isInvertible());
}

// ===================================================================
//  TEST 1: Cube consistency -- rigid body formula vs mesh integration
// ===================================================================
TEST_CASE("rigid_to_affine_cube", "[abd][rigid]")
{
    SimplicialComplexIO io;
    auto cube = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    Float rho = 1000.0;

    // --- Mesh-based computation ---
    Float     m_mesh;
    Vector3   mx_mesh;
    Matrix3x3 mxx_mesh;
    affine_body::compute_dyadic_mass(cube, rho, m_mesh, mx_mesh, mxx_mesh);
    Matrix12x12 M_mesh = build_abd_mass_matrix(m_mesh, mx_mesh, mxx_mesh);

    // --- Rigid body computation ---
    // For the cube mesh loaded from cube.msh, compute the rigid body
    // quantities from the mesh-based dyadic mass.
    // center of mass c = m_x_bar / m
    Vector3 c = mx_mesh / m_mesh;
    // Second moment S = m_x_bar_x_bar
    // Inertia about origin: I^O = tr(S)*I_3 - S
    Matrix3x3 I_origin = mxx_mesh.trace() * Matrix3x3::Identity() - mxx_mesh;
    // Inertia about CoM: I_cm = I^O - m*(|c|^2*I_3 - c*c^T)
    Matrix3x3 I_cm = I_origin - m_mesh * (c.squaredNorm() * Matrix3x3::Identity() - c * c.transpose());

    Matrix12x12 M_rigid = affine_body::from_rigid_body(m_mesh, c, I_cm);

    REQUIRE(M_rigid.isApprox(M_mesh, 1e-6));
    check_spd_invertible(M_rigid);
}

// ===================================================================
//  TEST 2: Diagonal inertia special case (c = 0)
// ===================================================================
TEST_CASE("rigid_to_affine_diagonal", "[abd][rigid]")
{
    Float Ix = 3.0;
    Float Iy = 5.0;
    Float Iz = 7.0;
    Float m  = 10.0;

    Vector3   c    = Vector3::Zero();
    Matrix3x3 I_cm = Matrix3x3::Zero();
    I_cm(0, 0) = Ix;
    I_cm(1, 1) = Iy;
    I_cm(2, 2) = Iz;

    Matrix12x12 M = affine_body::from_rigid_body(m, c, I_cm);

    // When c = 0, m_x_bar = 0, so off-diagonal blocks vanish.
    // S = diag((-Ix+Iy+Iz)/2, (Ix-Iy+Iz)/2, (Ix+Iy-Iz)/2)
    Float Sxx = (-Ix + Iy + Iz) / 2.0;
    Float Syy = ( Ix - Iy + Iz) / 2.0;
    Float Szz = ( Ix + Iy - Iz) / 2.0;

    // Translation block
    for(int i = 0; i < 3; ++i)
        REQUIRE(M(i, i) == Approx(m));

    // Off-diagonal coupling blocks should be zero (m_x_bar = 0)
    for(int i = 0; i < 3; ++i)
    {
        REQUIRE(M.block<1, 3>(i, 3 + 3 * i).norm() == Approx(0.0).margin(1e-12));
        REQUIRE(M.block<3, 1>(3 + 3 * i, i).norm() == Approx(0.0).margin(1e-12));
    }

    // A-block diagonals: each 3x3 block should be diag(Sxx, Syy, Szz)
    Matrix3x3 S_expected = Matrix3x3::Zero();
    S_expected(0, 0) = Sxx;
    S_expected(1, 1) = Syy;
    S_expected(2, 2) = Szz;

    for(int i = 0; i < 3; ++i)
        REQUIRE(M.block<3, 3>(3 + 3 * i, 3 + 3 * i).isApprox(S_expected, 1e-12));

    check_spd_invertible(M);
}

// ===================================================================
//  TEST 3: Roundtrip -- decompose a 12x12 matrix and reassemble
// ===================================================================
TEST_CASE("rigid_to_affine_roundtrip", "[abd][rigid]")
{
    SimplicialComplexIO io;
    auto cube = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    Float rho = 1000.0;

    Float     m;
    Vector3   m_x_bar;
    Matrix3x3 m_x_bar_x_bar;
    affine_body::compute_dyadic_mass(cube, rho, m, m_x_bar, m_x_bar_x_bar);

    Matrix12x12 M = build_abd_mass_matrix(m, m_x_bar, m_x_bar_x_bar);

    // Decompose -- same logic as apply_to(sc, kappa, mass, volume)
    Float     m2             = M(0, 0);
    Vector3   m_x_bar2       = M.block<3, 1>(3, 0);
    Matrix3x3 m_x_bar_x_bar2 = M.block<3, 3>(3, 3);

    REQUIRE(m2 == Approx(m));
    REQUIRE(m_x_bar2.isApprox(m_x_bar, 1e-12));
    REQUIRE(m_x_bar_x_bar2.isApprox(m_x_bar_x_bar, 1e-12));

    // Reassemble and compare
    Matrix12x12 M2 = build_abd_mass_matrix(m2, m_x_bar2, m_x_bar_x_bar2);
    REQUIRE(M2.isApprox(M, 1e-12));
}
