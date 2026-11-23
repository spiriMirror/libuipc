#include <app/app.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>
#include <uipc/constitution/elastic_moduli.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
Vector9 translate_triangle(const Vector9& X, const Vector3& shift)
{
    Vector9 out = X;
    for(int i = 0; i < 3; ++i)
        out.segment<3>(3 * i) += shift;
    return out;
}

Vector9 rotate_and_translate_triangle(const Vector9& X, const Matrix3x3& R, const Vector3& shift)
{
    Vector9 out;
    for(int i = 0; i < 3; ++i)
        out.segment<3>(3 * i) = R * X.segment<3>(3 * i) + shift;
    return out;
}
}  // namespace

TEST_CASE("neo_hookean_shell_2d_derivatives_and_off_origin_invariance", "[cuda][neo_hookean_shell_2d]")
{
    namespace NH = sym::neo_hookean_shell_2d;

    // Match fine thin-film triangle scale (~7 mm edge, A = 2.5e-5 m^2, E = 60 MPa, nu = 0.45)
    const Float h = std::sqrt(Float{5.0e-5});
    const auto moduli = constitution::ElasticModuli2D::youngs_poisson(6.0e7, 0.45);
    const Float lambda = moduli.lambda();
    const Float mu     = moduli.mu();

    Vector9 X_rest;
    X_rest << 0.0, 0.0, 0.0, h, 0.15 * h, 0.0, 0.1 * h, h, 0.0;

    Matrix2x2 B;
    NH::A(B, X_rest);
    const Matrix2x2 IB = B.inverse();

    // 1. Zero energy & zero gradient at rest state, including far from origin.
    for(const Vector3& shift : {Vector3{0.0, 0.0, 0.0},
                                Vector3{2.7013479, 1.7118969, 0.3941246},
                                Vector3{100.0, -50.0, 25.0}})
    {
        CAPTURE(shift.x(), shift.y(), shift.z());
        const Vector9 Xs = translate_triangle(X_rest, shift);
        Float         E  = 1.0;
        Vector9       G;
        NH::E(E, lambda, mu, Xs, IB);
        NH::dEdX(G, lambda, mu, Xs, IB);

        CHECK(std::isfinite(E));
        CHECK(G.allFinite());
        CHECK(std::abs(E) < 1e-6);
        CHECK(G.norm() < 1e-2);
        CHECK((G.segment<3>(0) + G.segment<3>(3) + G.segment<3>(6)).norm()
              < 1e-12 * (1.0 + G.norm()));
    }

    // 2. Analytical gradient & Hessian vs central finite differences + rigid invariance.
    Vector9 X_def = X_rest;
    X_def(3) += 0.04 * h;
    X_def(4) -= 0.02 * h;
    X_def(5) += 0.03 * h;
    X_def(6) -= 0.015 * h;
    X_def(7) += 0.05 * h;
    X_def(8) -= 0.025 * h;

    Float     E_origin = 0.0;
    Vector9   G_origin;
    Matrix9x9 H_origin;
    NH::E(E_origin, lambda, mu, X_def, IB);
    NH::dEdX(G_origin, lambda, mu, X_def, IB);
    NH::ddEddX(H_origin, lambda, mu, X_def, IB);

    const Matrix3x3 rot =
        Eigen::AngleAxis<Float>(0.73, Vector3{1.0, 2.0, -3.0}.normalized()).toRotationMatrix();

    for(const Vector3& shift : {Vector3{0.0, 0.0, 0.0},
                                Vector3{2.7013479, 1.7118969, 0.3941246},
                                Vector3{100.0, -50.0, 25.0}})
    {
        CAPTURE(shift.x(), shift.y(), shift.z());
        const Matrix3x3 R = (shift.norm() == 0.0) ? Matrix3x3::Identity() : rot;
        const Vector9   X = rotate_and_translate_triangle(X_def, R, shift);

        Float     E = 0.0;
        Vector9   G;
        Matrix9x9 H;
        NH::E(E, lambda, mu, X, IB);
        NH::dEdX(G, lambda, mu, X, IB);
        NH::ddEddX(H, lambda, mu, X, IB);

        CHECK(std::isfinite(E));
        CHECK(G.allFinite());
        CHECK(H.allFinite());
        CHECK(std::abs(E - E_origin) <= 1e-9 * std::abs(E_origin));

        Vector9 G_expected;
        for(int i = 0; i < 3; ++i)
            G_expected.segment<3>(3 * i) = R * G_origin.segment<3>(3 * i);
        CHECK((G - G_expected).norm() <= 1e-9 * G_expected.norm());

        Matrix9x9 H_expected;
        for(int br = 0; br < 3; ++br)
            for(int bc = 0; bc < 3; ++bc)
                H_expected.block<3, 3>(3 * br, 3 * bc) =
                    R * H_origin.block<3, 3>(3 * br, 3 * bc) * R.transpose();
        CHECK((H - H_expected).norm() <= 1e-9 * H_expected.norm());

        // Symmetry and translation nullspace
        CHECK((H - H.transpose()).norm() <= 1e-12 * H.norm());
        CHECK((G.segment<3>(0) + G.segment<3>(3) + G.segment<3>(6)).norm()
              <= 1e-12 * G.norm());
        for(int bc = 0; bc < 3; ++bc)
        {
            const Matrix3x3 col_sum = H.block<3, 3>(0, 3 * bc) + H.block<3, 3>(3, 3 * bc)
                                      + H.block<3, 3>(6, 3 * bc);
            CHECK(col_sum.norm() <= 1e-12 * H.norm());
        }

        // Central finite differences directly at world coordinates
        const Float step = 1e-7 * h;
        Vector9     G_fd;
        Matrix9x9   H_fd;
        for(int i = 0; i < 9; ++i)
        {
            Vector9 Xp = X, Xm = X;
            Xp(i) += step;
            Xm(i) -= step;
            Float Ep = 0.0, Em = 0.0;
            NH::E(Ep, lambda, mu, Xp, IB);
            NH::E(Em, lambda, mu, Xm, IB);
            G_fd(i) = (Ep - Em) / (Float{2.0} * step);

            Vector9 Gp, Gm;
            NH::dEdX(Gp, lambda, mu, Xp, IB);
            NH::dEdX(Gm, lambda, mu, Xm, IB);
            H_fd.col(i) = (Gp - Gm) / (Float{2.0} * step);
        }
        CHECK((G - G_fd).norm() <= 1e-5 * G.norm());
        CHECK((H - H_fd).norm() <= 1e-5 * H.norm());
    }
}
