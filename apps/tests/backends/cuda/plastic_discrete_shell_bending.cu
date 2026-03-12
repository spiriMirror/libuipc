#include <app/app.h>
#include <finite_element/constitutions/plastic_discrete_shell_bending_function.h>
#include <cmath>
#include <limits>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace PDSB = sym::plastic_discrete_shell_bending;

TEST_CASE("plastic_discrete_shell_bending_helpers", "[cuda][plastic_dsb]")
{
    SECTION("wrap_angle")
    {
        constexpr Float Pi = PDSB::pi<Float>();
        CHECK(PDSB::wrap_angle<Float>(1.5 * Pi) == Catch::Approx(-0.5 * Pi).margin(1e-12));
        CHECK(PDSB::angle_delta<Float>(Pi - 0.1, -Pi + 0.1)
              == Catch::Approx(-0.2).margin(1e-12));
    }

    SECTION("below_yield_does_not_update")
    {
        Float theta_bar       = 0.0;
        Float yield_threshold = 0.5;
        const bool updated =
            PDSB::update_plastic_state<Float>(0.25, theta_bar, yield_threshold, 0.0);

        CHECK_FALSE(updated);
        CHECK(theta_bar == Catch::Approx(0.0));
        CHECK(yield_threshold == Catch::Approx(0.5));
    }

    SECTION("positive_yielding_updates_theta_bar")
    {
        Float theta_bar       = 0.0;
        Float yield_threshold = 0.5;
        const bool updated =
            PDSB::update_plastic_state<Float>(0.8, theta_bar, yield_threshold, 0.0);

        REQUIRE(updated);
        CHECK(theta_bar == Catch::Approx(0.3).margin(1e-12));
        CHECK(yield_threshold == Catch::Approx(0.5).margin(1e-12));
    }

    SECTION("negative_yielding_updates_theta_bar")
    {
        Float theta_bar       = 0.0;
        Float yield_threshold = 0.5;
        const bool updated =
            PDSB::update_plastic_state<Float>(-0.8, theta_bar, yield_threshold, 0.0);

        REQUIRE(updated);
        CHECK(theta_bar == Catch::Approx(-0.3).margin(1e-12));
        CHECK(yield_threshold == Catch::Approx(0.5).margin(1e-12));
    }

    SECTION("hardening_increases_yield_threshold")
    {
        Float theta_bar       = 0.0;
        Float yield_threshold = 0.5;
        const bool updated =
            PDSB::update_plastic_state<Float>(0.8, theta_bar, yield_threshold, 2.0);

        REQUIRE(updated);
        CHECK(theta_bar == Catch::Approx(0.3).margin(1e-12));
        CHECK(yield_threshold == Catch::Approx(1.1).margin(1e-12));
    }

    SECTION("non_finite_inputs_are_rejected")
    {
        Float theta_bar       = 0.0;
        Float yield_threshold = 0.5;
        const Float nan       = std::numeric_limits<Float>::quiet_NaN();

        CHECK_FALSE(PDSB::update_plastic_state<Float>(nan, theta_bar, yield_threshold, 0.0));
        CHECK(theta_bar == Catch::Approx(0.0));
        CHECK(yield_threshold == Catch::Approx(0.5));
    }

    SECTION("degenerate_hinge_is_rejected")
    {
        Float theta = 0.0;
        CHECK_FALSE(PDSB::safe_dihedral_angle(Vector3{0.0, 0.0, 0.0},
                                              Vector3{1.0, 0.0, 0.0},
                                              Vector3{2.0, 0.0, 0.0},
                                              Vector3{3.0, 0.0, 0.0},
                                              theta));
    }

    SECTION("simple_hinge_is_accepted")
    {
        Float theta = 0.0;
        REQUIRE(PDSB::safe_dihedral_angle(Vector3{0.0, 0.0, 0.0},
                                          Vector3{0.0, 0.0, 1.0},
                                          Vector3{1.0, 0.0, 0.0},
                                          Vector3{1.0, 1.0, 1.0},
                                          theta));
        CHECK(std::isfinite(theta));
    }

    SECTION("safe_dihedral_matches_reference_on_valid_hinge")
    {
        const Vector3 x0{0.0, 0.0, 0.0};
        const Vector3 x1{0.0, 0.0, 1.0};
        const Vector3 x2{1.0, 0.0, 0.0};
        const Vector3 x3{1.0, 1.0, 1.0};

        Float safe_theta      = 0.0;
        Float reference_theta = 0.0;
        REQUIRE(PDSB::safe_dihedral_angle(x0, x1, x2, x3, safe_theta));
        dihedral_angle(x0, x1, x2, x3, reference_theta);

        CHECK(safe_theta == Catch::Approx(reference_theta).margin(1e-12));
    }

    SECTION("degenerate_hinge_zeroes_energy_gradient_and_hessian")
    {
        const Vector3 x0{0.0, 0.0, 0.0};
        const Vector3 x1{1.0, 0.0, 0.0};
        const Vector3 x2{2.0, 0.0, 0.0};
        const Vector3 x3{3.0, 0.0, 0.0};

        const Float E = PDSB::E(x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0);
        CHECK(E == Catch::Approx(0.0).margin(1e-12));

        Vector12 G = Vector12::Ones();
        PDSB::dEdx(G, x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0);
        CHECK(G.norm() == Catch::Approx(0.0).margin(1e-12));

        Matrix12x12 H = Matrix12x12::Ones();
        PDSB::ddEddx(H, x0, x1, x2, x3, 1.0, 1.0, 0.0, 1.0);
        CHECK(H.cwiseAbs().maxCoeff() == Catch::Approx(0.0).margin(1e-12));
    }
}
