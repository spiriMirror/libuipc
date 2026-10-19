#include <app/app.h>
#include <finite_element/constitutions/discrete_shell_bending_function.h>
#include <uipc/constitution/discrete_shell_bending.h>

using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("discrete_shell_bending_physical_calibration", "[cuda][discrete_shell_bending]")
{
    namespace DSB = sym::discrete_shell_bending;

    const Vector3 x0{0.0, 1.0, 0.0};
    const Vector3 x1{0.0, 0.0, 0.0};
    const Vector3 x2{1.0, 0.0, 0.0};
    const Vector3 x3{1.0, -1.0, 0.0};
    Float         L0, h_bar, theta_bar, outer_weight;
    detail::compute_discrete_shell_bending_reference(
        L0, h_bar, theta_bar, outer_weight, x0, x1, x2, x3);

    // E=288000 Pa, nu=0.2, full h=0.02 m give plate rigidity D=0.2 N*m.
    // This hinge has 3*L0^2/(A1+A2)=3. At delta_theta=0.25 rad, the
    // plate-calibrated energy is (D/2)*3*0.25^2 = 0.01875 J.
    const Float kappa =
        constitution::DiscreteShellBending::bending_stiffness(288000.0, 0.2, 0.01);
    const Float theta = theta_bar + 0.25;
    Float       energy, torque, tangent;
    DSB::E(energy, kappa, theta, theta_bar, L0, h_bar);
    DSB::dEdtheta(torque, kappa, theta, theta_bar, L0, h_bar);
    DSB::ddEddtheta(tangent, kappa, theta, theta_bar, L0, h_bar);
    CHECK(outer_weight * energy == Catch::Approx(0.01875));
    CHECK(outer_weight * torque == Catch::Approx(0.15));
    CHECK(outer_weight * tangent == Catch::Approx(0.6));
}

TEST_CASE("discrete_shell_bending_spatial_derivatives", "[cuda][discrete_shell_bending]")
{
    namespace DSB = sym::discrete_shell_bending;
    for(Float rest_angle : {0.0, 0.35})
    {
        for(Float fold : {-0.6, 0.7})
        {
            CAPTURE(rest_angle, fold);
            const Vector3 x0{0.0, 1.0, 0.0};
            const Vector3 x1{0.0, 0.0, 0.0};
            const Vector3 x2{1.0, 0.0, 0.0};
            const Vector3 x3{1.0, -std::cos(rest_angle), std::sin(rest_angle)};
            Float         L0, h_bar, theta_bar, outer_weight;
            detail::compute_discrete_shell_bending_reference(
                L0, h_bar, theta_bar, outer_weight, x0, x1, x2, x3);
            CHECK(theta_bar == Catch::Approx(-rest_angle).margin(1e-12));
            constexpr Float kappa = 0.1;
            CHECK(DSB::E(x0, x1, x2, x3, L0, h_bar, theta_bar, kappa)
                  == Catch::Approx(0.0).margin(1e-14));
            Vector12 rest_gradient;
            DSB::dEdx(rest_gradient, x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
            CHECK(rest_gradient.norm() < 1e-12);

            Vector12 x;
            x << 0.12, 1.2, 0.2, -0.03, 0.04, -0.05, 1.1, 0.1, 0.0, 1.0,
                -std::cos(fold), std::sin(fold);
            auto energy = [&](const Vector12& p)
            {
                return DSB::E(p.segment<3>(0),
                              p.segment<3>(3),
                              p.segment<3>(6),
                              p.segment<3>(9),
                              L0,
                              h_bar,
                              theta_bar,
                              kappa);
            };
            auto gradient = [&](const Vector12& p)
            {
                Vector12 result;
                DSB::dEdx(result,
                          p.segment<3>(0),
                          p.segment<3>(3),
                          p.segment<3>(6),
                          p.segment<3>(9),
                          L0,
                          h_bar,
                          theta_bar,
                          kappa);
                return result;
            };

            // Independently compute the signed angle using atan2. The runtime
            // uses acos plus a sign; the two agree away from the branch cut.
            const Vector3 p0 = x.segment<3>(0), p1 = x.segment<3>(3);
            const Vector3 p2 = x.segment<3>(6), p3 = x.segment<3>(9);
            const Vector3 n1 = (p1 - p0).cross(p2 - p0).normalized();
            const Vector3 n2 = (p2 - p3).cross(p1 - p3).normalized();
            const Float   theta =
                std::atan2((p2 - p1).normalized().dot(n1.cross(n2)), n1.dot(n2));
            CHECK(energy(x) == Catch::Approx(3 * kappa * std::pow(theta + rest_angle, 2)));

            Vector12    numerical_gradient;
            Matrix12x12 numerical_hessian, hessian;
            DSB::ddEddx(hessian, p0, p1, p2, p3, L0, h_bar, theta_bar, kappa);
            constexpr Float step = 1e-6;
            for(int i = 0; i < 12; ++i)
            {
                Vector12 plus = x, minus = x;
                plus(i) += step;
                minus(i) -= step;
                numerical_gradient(i) = (energy(plus) - energy(minus)) / (2 * step);
                numerical_hessian.col(i) = (gradient(plus) - gradient(minus)) / (2 * step);
            }
            const Vector12 g = gradient(x);
            CHECK((g - numerical_gradient).norm() < 1e-6 * g.norm());
            CHECK((hessian - numerical_hessian).norm() < 1e-6 * hessian.norm());
            CHECK((hessian - hessian.transpose()).norm() < 1e-12);
            CHECK((g.segment<3>(0) + g.segment<3>(3) + g.segment<3>(6) + g.segment<3>(9))
                      .norm()
                  < 1e-12);

            const Matrix3x3 rotation =
                Eigen::AngleAxis<Float>(0.7, Vector3{1, 2, 3}.normalized()).toRotationMatrix();
            Vector12 moved;
            for(int i = 0; i < 4; ++i)
                moved.segment<3>(3 * i) =
                    rotation * x.segment<3>(3 * i) + Vector3{3, -5, 0.7};
            CHECK(energy(moved) == Catch::Approx(energy(x)));
        }
    }
}
