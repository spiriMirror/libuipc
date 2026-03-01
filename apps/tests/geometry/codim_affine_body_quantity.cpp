/**
 * Unit tests for codim (shell/rod) affine body dyadic mass computation.
 *
 * The 12x12 ABD mass matrix is assembled from (m, m_x_bar, m_x_bar_x_bar)
 * using the same structure as ABDJacobiDyadicMass::add_to().
 *
 * Test coverage:
 *   - Axis-aligned triangle / segment with analytical value checks
 *   - Tilted triangle (normal not axis-aligned)
 *   - Tilted segment (tangent not axis-aligned)
 *   - Offset geometry (not at origin)
 *   - Multiple elements (2 triangles, 3 segments)
 *   - Very thin shell / rod (small r)
 *
 * Every case checks: symmetry, positive-definiteness (LLT), invertibility.
 */

#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <numbers>

using namespace uipc;
using namespace uipc::geometry;
using Catch::Approx;

// ---------------------------------------------------------------------------
// Helper: build the 12x12 ABD mass matrix from the three dyadic quantities.
// Mirrors ABDJacobiDyadicMass::add_to() from the CUDA backend.
// ---------------------------------------------------------------------------
static Matrix12x12 build_abd_mass_matrix(Float             m,
                                         const Vector3&    m_x_bar,
                                         const Matrix3x3&  m_x_bar_x_bar)
{
    Matrix12x12 M = Matrix12x12::Zero();

    for(int i = 0; i < 3; ++i)
    {
        M(i, i) += m;
        M.block<1, 3>(i, 3 + 3 * i) += m_x_bar.transpose();
        M.block<3, 1>(3 + 3 * i, i) += m_x_bar;
        M.block<3, 3>(3 + 3 * i, 3 + 3 * i) += m_x_bar_x_bar;
    }

    return M;
}

// ---------------------------------------------------------------------------
// Helper: check that the 12x12 mass matrix is symmetric, positive-definite,
// and invertible.  Used by every test section.
// ---------------------------------------------------------------------------
static void check_spd_invertible(const Matrix12x12& M)
{
    // Symmetric
    REQUIRE(M.isApprox(M.transpose(), 1e-9));

    // Positive definite (Cholesky succeeds)
    Eigen::LLT<Matrix12x12> llt(M);
    REQUIRE(llt.info() == Eigen::Success);

    // Full-rank
    REQUIRE(M.fullPivLu().isInvertible());
}

// ===================================================================
//  S H E L L   T E S T S
// ===================================================================

TEST_CASE("codim_abd_shell_axis_aligned", "[abd][codim]")
{
    // Triangle in z = 0 plane: p0=(0,0,0), p1=(1,0,0), p2=(0,1,0)
    vector<Vector3>  Vs = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    vector<Vector3i> Fs = {Vector3i{0, 1, 2}};
    auto sc = trimesh(Vs, Fs);

    const Float rho  = 1000.0;
    const Float r    = 0.1;
    const Float twor = 2.0 * r;

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, rho, r, m, mx, mxx);

    // Analytical mass
    REQUIRE(m == Approx(rho * 0.5 * twor).epsilon(1e-6));

    // Analytical centroid
    Vector3 expected_mx = m * Vector3(1.0 / 3, 1.0 / 3, 0);
    REQUIRE(mx.isApprox(expected_mx, 1e-6));

    // Second moment: in-plane terms + normal correction
    const Float D = 1.0;
    Matrix3x3 expected_mxx = Matrix3x3::Zero();
    expected_mxx(0, 0) = rho * D * twor / 12.0;
    expected_mxx(1, 1) = rho * D * twor / 12.0;
    expected_mxx(0, 1) = rho * D * twor / 24.0;
    expected_mxx(1, 0) = expected_mxx(0, 1);
    expected_mxx(2, 2) = rho * 0.5 * (2.0 * r * r * r / 3.0);
    REQUIRE(mxx.isApprox(expected_mxx, 1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_shell_tilted", "[abd][codim]")
{
    // Triangle with normal = (1,1,1)/sqrt(3)
    // p0 = (0,0,0), p1 = (1,0,-1), p2 = (0,1,-1)
    // e1 = (1,0,-1), e2 = (0,1,-1)
    // e1 x e2 = (0*(-1)-(-1)*1, (-1)*0-1*(-1), 1*1-0*0) = (1, 1, 1)
    vector<Vector3>  Vs = {{0, 0, 0}, {1, 0, -1}, {0, 1, -1}};
    vector<Vector3i> Fs = {Vector3i{0, 1, 2}};
    auto sc = trimesh(Vs, Fs);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 0.1, m, mx, mxx);

    // m_x_bar_x_bar must be full-rank (normal correction fills all 3 dirs)
    REQUIRE(Eigen::FullPivLU<Matrix3x3>(mxx).rank() == 3);

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_shell_offset", "[abd][codim]")
{
    // Same triangle shape, but offset to (5, 3, 7)
    Vector3 offset{5.0, 3.0, 7.0};
    vector<Vector3>  Vs = {offset, offset + Vector3{1, 0, 0}, offset + Vector3{0, 1, 0}};
    vector<Vector3i> Fs = {Vector3i{0, 1, 2}};
    auto sc = trimesh(Vs, Fs);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 0.05, m, mx, mxx);

    // Centroid check
    Vector3 centroid = (Vs[0] + Vs[1] + Vs[2]) / 3.0;
    REQUIRE(mx.isApprox(m * centroid, 1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_shell_multi_triangle", "[abd][codim]")
{
    // Two triangles forming a quad: (0,0,0)-(1,0,0)-(1,1,0)-(0,1,0)
    vector<Vector3> Vs = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    vector<Vector3i> Fs = {Vector3i{0, 1, 2}, Vector3i{0, 2, 3}};
    auto sc = trimesh(Vs, Fs);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 500.0, 0.2, m, mx, mxx);

    // Total area = 1.0, effective volume = 1.0 * 2*0.2 = 0.4
    REQUIRE(m == Approx(500.0 * 1.0 * 0.4).epsilon(1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_shell_thin", "[abd][codim]")
{
    // Very thin shell: r = 1e-4 (should still be invertible)
    vector<Vector3>  Vs = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    vector<Vector3i> Fs = {Vector3i{0, 1, 2}};
    auto sc = trimesh(Vs, Fs);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 1e-4, m, mx, mxx);

    // Must still be full-rank
    REQUIRE(Eigen::FullPivLU<Matrix3x3>(mxx).rank() == 3);
    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}


// ===================================================================
//  R O D   T E S T S
// ===================================================================

TEST_CASE("codim_abd_rod_axis_aligned", "[abd][codim]")
{
    // Segment along x-axis: p0=(0,0,0), p1=(1,0,0)
    vector<Vector3>  Vs = {{0, 0, 0}, {1, 0, 0}};
    vector<Vector2i> Es = {Vector2i{0, 1}};
    auto sc = linemesh(Vs, Es);

    const Float rho = 1000.0;
    const Float r   = 0.1;
    const Float pi  = std::numbers::pi_v<Float>;
    const Float S   = pi * r * r;
    const Float Ics = pi * r * r * r * r / 4.0;

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, rho, r, m, mx, mxx);

    // Analytical mass
    REQUIRE(m == Approx(rho * 1.0 * S).epsilon(1e-6));

    // Analytical midpoint
    REQUIRE(mx.isApprox(m * Vector3(0.5, 0, 0), 1e-6));

    // Second moment
    Matrix3x3 expected_mxx = Matrix3x3::Zero();
    expected_mxx(0, 0) = rho * S * (1.0 / 3.0);        // along-axis
    expected_mxx(1, 1) = rho * 1.0 * Ics;               // cross-section
    expected_mxx(2, 2) = rho * 1.0 * Ics;               // cross-section
    REQUIRE(mxx.isApprox(expected_mxx, 1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_rod_tilted", "[abd][codim]")
{
    // Segment along (1,1,1)/sqrt(3) direction
    Float len = 2.0;
    Vector3 dir = Vector3(1, 1, 1).normalized();
    vector<Vector3>  Vs = {{0, 0, 0}, {len * dir[0], len * dir[1], len * dir[2]}};
    vector<Vector2i> Es = {Vector2i{0, 1}};
    auto sc = linemesh(Vs, Es);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 0.1, m, mx, mxx);

    // mxx must be full-rank (cross-section correction fills perpendicular dirs)
    REQUIRE(Eigen::FullPivLU<Matrix3x3>(mxx).rank() == 3);

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_rod_offset", "[abd][codim]")
{
    // Segment offset away from origin
    Vector3 offset{3.0, -2.0, 5.0};
    vector<Vector3>  Vs = {offset, offset + Vector3{0, 0, 1.5}};
    vector<Vector2i> Es = {Vector2i{0, 1}};
    auto sc = linemesh(Vs, Es);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 2000.0, 0.05, m, mx, mxx);

    // Midpoint check
    Vector3 midpoint = (Vs[0] + Vs[1]) / 2.0;
    REQUIRE(mx.isApprox(m * midpoint, 1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_rod_multi_segment", "[abd][codim]")
{
    // Polyline: 4 vertices, 3 segments, L-shaped
    vector<Vector3> Vs = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}};
    vector<Vector2i> Es = {Vector2i{0, 1}, Vector2i{1, 2}, Vector2i{2, 3}};
    auto sc = linemesh(Vs, Es);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 0.1, m, mx, mxx);

    // Total length = 3, effective volume = 3 * pi * 0.01
    const Float pi = std::numbers::pi_v<Float>;
    REQUIRE(m == Approx(1000.0 * 3.0 * pi * 0.01).epsilon(1e-6));

    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

TEST_CASE("codim_abd_rod_thin", "[abd][codim]")
{
    // Very thin rod: r = 1e-4 (should still be invertible)
    vector<Vector3>  Vs = {{0, 0, 0}, {1, 0, 0}};
    vector<Vector2i> Es = {Vector2i{0, 1}};
    auto sc = linemesh(Vs, Es);

    Float     m;
    Vector3   mx;
    Matrix3x3 mxx;
    affine_body::compute_dyadic_mass(sc, 1000.0, 1e-4, m, mx, mxx);

    REQUIRE(Eigen::FullPivLU<Matrix3x3>(mxx).rank() == 3);
    check_spd_invertible(build_abd_mass_matrix(m, mx, mxx));
}

