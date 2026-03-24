#include <app/app.h>
#include <finite_element/constitutions/stress_plastic_discrete_shell_bending_function.h>
#include <cmath>
#include <limits>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace SPDSB = sym::stress_plastic_discrete_shell_bending;

TEST_CASE("stress_plastic_discrete_shell_bending_helpers", "[cuda][stress_plastic_dsb]")
{
    SECTION("wrap_angle")
    {
        constexpr Float Pi = SPDSB::pi<Float>();
        CHECK(SPDSB::wrap_angle<Float>(1.5 * Pi) == Catch::Approx(-0.5 * Pi).margin(1e-12));
        CHECK(SPDSB::angle_delta<Float>(Pi - 0.1, -Pi + 0.1)
              == Catch::Approx(-0.2).margin(1e-12));
    }

    SECTION("elastic_response_matches_quadratic_energy")
    {
        Float energy     = 0.0;
        Float dEdtheta   = 0.0;
        Float ddEddtheta = 0.0;

        REQUIRE(SPDSB::augmented_response_from_angle_delta<Float>(
            0.25, 2.0, 3.0, 4.0, 1.2, energy, dEdtheta, ddEddtheta));
        CHECK(energy == Catch::Approx(0.09375).margin(1e-12));
        CHECK(dEdtheta == Catch::Approx(0.75).margin(1e-12));
        CHECK(ddEddtheta == Catch::Approx(3.0).margin(1e-12));
    }

    SECTION("plastic_response_saturates_stress_and_zeroes_scalar_curvature")
    {
        Float energy     = 0.0;
        Float dEdtheta   = 0.0;
        Float ddEddtheta = 0.0;

        REQUIRE(SPDSB::augmented_response_from_angle_delta<Float>(
            0.6, 2.0, 3.0, 4.0, 1.2, energy, dEdtheta, ddEddtheta));
        CHECK(energy == Catch::Approx(0.48).margin(1e-12));
        CHECK(dEdtheta == Catch::Approx(1.2).margin(1e-12));
        CHECK(ddEddtheta == Catch::Approx(0.0).margin(1e-12));
    }

    SECTION("positive_yielding_updates_theta_bar_and_yield_stress")
    {
        Float theta_bar    = 0.0;
        Float yield_stress = 1.2;

        const bool updated = SPDSB::update_plastic_state<Float>(
            0.6, theta_bar, yield_stress, 2.0, 2.0, 3.0, 4.0);

        REQUIRE(updated);
        CHECK(theta_bar == Catch::Approx(0.2).margin(1e-12));
        CHECK(yield_stress == Catch::Approx(1.6).margin(1e-12));
    }

    SECTION("below_yield_does_not_update")
    {
        Float theta_bar    = 0.0;
        Float yield_stress = 1.2;

        const bool updated = SPDSB::update_plastic_state<Float>(
            0.25, theta_bar, yield_stress, 2.0, 2.0, 3.0, 4.0);

        CHECK_FALSE(updated);
        CHECK(theta_bar == Catch::Approx(0.0));
        CHECK(yield_stress == Catch::Approx(1.2));
    }

    SECTION("negative_yielding_updates_theta_bar")
    {
        Float theta_bar    = 0.0;
        Float yield_stress = 1.2;

        const bool updated = SPDSB::update_plastic_state<Float>(
            -0.6, theta_bar, yield_stress, 0.0, 2.0, 3.0, 4.0);

        REQUIRE(updated);
        CHECK(theta_bar == Catch::Approx(-0.2).margin(1e-12));
        CHECK(yield_stress == Catch::Approx(1.2).margin(1e-12));
    }

    SECTION("non_finite_inputs_are_rejected")
    {
        Float theta_bar    = 0.0;
        Float yield_stress = 1.2;
        const Float nan    = std::numeric_limits<Float>::quiet_NaN();

        CHECK_FALSE(SPDSB::update_plastic_state<Float>(
            nan, theta_bar, yield_stress, 0.0, 2.0, 3.0, 4.0));
        CHECK(theta_bar == Catch::Approx(0.0));
        CHECK(yield_stress == Catch::Approx(1.2));
    }

    SECTION("degenerate_hinge_is_rejected")
    {
        Float theta = 0.0;
        CHECK_FALSE(SPDSB::safe_dihedral_angle(Vector3{0.0, 0.0, 0.0},
                                               Vector3{1.0, 0.0, 0.0},
                                               Vector3{2.0, 0.0, 0.0},
                                               Vector3{3.0, 0.0, 0.0},
                                               theta));
    }

    SECTION("degenerate_hinge_zeroes_energy_gradient_and_hessian")
    {
        const Vector3 x0{0.0, 0.0, 0.0};
        const Vector3 x1{1.0, 0.0, 0.0};
        const Vector3 x2{2.0, 0.0, 0.0};
        const Vector3 x3{3.0, 0.0, 0.0};

        const Float E = SPDSB::E(x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0, 0.5);
        CHECK(E == Catch::Approx(0.0).margin(1e-12));

        Vector12 G = Vector12::Ones();
        SPDSB::dEdx(G, x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0, 0.5);
        CHECK(G.norm() == Catch::Approx(0.0).margin(1e-12));

        Matrix12x12 H = Matrix12x12::Ones();
        SPDSB::ddEddx(H, x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0, 0.5);
        CHECK(H.cwiseAbs().maxCoeff() == Catch::Approx(0.0).margin(1e-12));
    }
}
