#include <finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.h>
#include <app/app.h>
#include <Eigen/Eigenvalues>

using namespace uipc;
using namespace uipc::backend::cuda;
namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

namespace
{
using FMatrix = Matrix<Float, 3, 2>;

Matrix6x6 positive_part(const Matrix6x6& H)
{
    Eigen::SelfAdjointEigenSolver<Matrix6x6> solver(H);
    return solver.eigenvectors() * solver.eigenvalues().cwiseMax(0).asDiagonal()
           * solver.eigenvectors().transpose();
}

// Independent reference: differentiate each radial stretch potential, then
// project each stretch term and the shear term separately, as in the solver.
Matrix6x6 reference_hessian(const FMatrix& F, Float ka, Float kb, Float kg, Float rate)
{
    Matrix6x6 result = Matrix6x6::Zero();
    for(int axis = 0; axis < 2; ++axis)
    {
        const Vector3 f    = F.col(axis);
        Float         s    = f.norm();
        Float         e    = s - 1;
        Float         d1   = 2 * e + (e > 0 ? 3 * rate * e * e : 0);
        Float         d2   = 2 + (e > 0 ? 6 * rate * e : 0);
        Matrix6x6     term = Matrix6x6::Zero();
        term.block<3, 3>(3 * axis, 3 * axis) =
            (d1 / s) * Matrix3x3::Identity()
            + (d2 - d1 / s) * (f * f.transpose()) / (s * s);
        result += (axis == 0 ? ka : kb) * positive_part(term);
    }
    Vector6 g;
    g << F.col(1), F.col(0);
    Matrix6x6 cross = Matrix6x6::Zero();
    cross.block<3, 3>(0, 3).setIdentity();
    cross.block<3, 3>(3, 0).setIdentity();
    result += kg * positive_part(2 * (g * g.transpose() + F.col(0).dot(F.col(1)) * cross));
    return result;
}
}  // namespace

TEST_CASE("baraff_witkin_directional_energy_and_derivatives", "[cuda][baraff_witkin]")
{
    const Vector2 a(1, 0), b(0, 1);
    for(Float rate : {0.0, 100.0})
    {
        for(Float kb : {17.0, 83.0})
        {
            const Float ka = 83, kg = 3.5;
            for(Float scale : {0.75, 1.0, 1.25})
            {
                CAPTURE(rate, ka, kb, scale);
                FMatrix F;
                F << scale, 0.15, 0.07, 1.1, -0.1, 0.2;
                const auto phi = [&](Float s)
                {
                    const Float extension = std::max(s - 1, Float{0});
                    return (s - 1) * (s - 1) + rate * extension * extension * extension;
                };
                Float expected = ka * phi(F.col(0).norm()) + kb * phi(F.col(1).norm())
                                 + kg * std::pow(F.col(0).dot(F.col(1)), 2);
                CHECK(BWS::E(F, a, b, ka, kb, kg, rate)
                      == Catch::Approx(expected).epsilon(1e-12));
                FMatrix P;
                BWS::dEdF(P, F, a, b, ka, kb, kg, rate);
                for(int i = 0; i < 6; ++i)
                {
                    constexpr Float step = 1e-6;
                    FMatrix         plus = F, minus = F;
                    plus.data()[i] += step;
                    minus.data()[i] -= step;
                    Float numeric = (BWS::E(plus, a, b, ka, kb, kg, rate)
                                     - BWS::E(minus, a, b, ka, kb, kg, rate))
                                    / (2 * step);
                    CHECK(P.data()[i] == Catch::Approx(numeric).epsilon(1e-6).margin(1e-6));
                }
                Matrix6x6 H;
                BWS::ddEddF(H, F, a, b, ka, kb, kg, rate);
                CHECK((H - reference_hessian(F, ka, kb, kg, rate)).norm()
                      < 1e-10 * H.norm());
                CHECK((H - H.transpose()).norm() < 1e-10 * H.norm());
                Eigen::SelfAdjointEigenSolver<Matrix6x6> solver(H);
                CHECK(solver.eigenvalues().minCoeff() >= -1e-10 * H.norm());
            }
        }
    }
}

TEST_CASE("baraff_witkin_independent_axis_response", "[cuda][baraff_witkin]")
{
    const Vector2 a(1, 0), b(0, 1);
    FMatrix       rest;
    rest << 1, 0, 0, 1, 0, 0;
    for(int axis = 0; axis < 2; ++axis)
    {
        for(Float stretch : {0.8, 1.0, 1.2})
        {
            FMatrix F = rest;
            F.col(axis) *= stretch;
            const Float other     = 7;
            const Float stiffness = axis == 0 ? 83 : 17;
            const Float ka        = axis == 0 ? stiffness : other;
            const Float kb        = axis == 1 ? stiffness : other;
            Float       e         = stretch - 1;
            Float       expected =
                stiffness * (e * e + 100 * std::pow(std::max(e, Float{0}), 3));
            CHECK(BWS::E(F, a, b, ka, kb, 3.5, 100) == Catch::Approx(expected).margin(1e-12));
            FMatrix P;
            BWS::dEdF(P, F, a, b, ka, kb, 3.5, 100);
            CHECK(P.col(1 - axis).norm() < 1e-12);
            CHECK(P(axis, axis)
                  == Catch::Approx(stiffness
                                   * (2 * e + 300 * std::pow(std::max(e, Float{0}), 2)))
                         .margin(1e-10));
            Matrix6x6 H;
            BWS::ddEddF(H, F, a, b, ka, kb, 3.5, 100);
            CHECK((H - reference_hessian(F, ka, kb, 3.5, 100)).norm() < 1e-10 * H.norm());
        }
    }
}
