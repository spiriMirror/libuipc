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
