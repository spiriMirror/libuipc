#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <uipc/constitution/strain_limiting_baraff_witkin.h>
#include <algorithm>

TEST_CASE("strain_limiting_baraff_witkin_attribute_layout", "[constitution]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    vector<Vector3> Vs = {
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 1.0},
        Vector3{0.0, 0.0, 1.0},
    };
    vector<Vector3i> Fs = {
        Vector3i{0, 1, 2},
        Vector3i{0, 2, 3},
    };

    auto mesh = trimesh(Vs, Fs);

    StrainLimitingBaraffWitkinShell slbws;

    // separate stretch / shear moduli
    Float E_stretch = 1.0_MPa, nu_stretch = 0.49;
    Float E_shear = 10.0_kPa, nu_shear = 0.4;
    Float thickness  = 0.001_m;
    Float strain_rate = 120.0;
    slbws.apply_to(mesh,
                   ElasticModuli2D::youngs_poisson(E_stretch, nu_stretch),
                   ElasticModuli2D::youngs_poisson(E_shear, nu_shear),
                   200.0,
                   thickness,
                   strain_rate);

    auto lambda = mesh.triangles().find<Float>("lambda");
    auto mu     = mesh.triangles().find<Float>("mu");
    auto sr     = mesh.triangles().find<Float>("strain_rate");

    REQUIRE(lambda);
    REQUIRE(mu);
    REQUIRE(sr);
    CHECK(lambda->size() == mesh.triangles().size());
    CHECK(mu->size() == mesh.triangles().size());
    CHECK(sr->size() == mesh.triangles().size());

    // stretch attr = E_stretch * t / (1 - nu_stretch^2)  [= (lambda + 2*mu) * t]
    Float expect_stretch =
        E_stretch * thickness / (1.0 - nu_stretch * nu_stretch);
    // shear attr = E_shear / (2 * (1 + nu_shear))  (thickness-independent)
    Float expect_shear = E_shear / (2.0 * (1.0 + nu_shear));

    CHECK(std::ranges::all_of(
        lambda->view(),
        [&](Float v) { return v == Catch::Approx(expect_stretch); }));
    CHECK(std::ranges::all_of(
        mu->view(), [&](Float v) { return v == Catch::Approx(expect_shear); }));
    CHECK(std::ranges::all_of(
        sr->view(), [&](Float v) { return v == Catch::Approx(strain_rate); }));

    // shared-moduli overload: stretch and shear derive from the same pair
    auto mesh2 = trimesh(Vs, Fs);
    slbws.apply_to(mesh2, ElasticModuli2D::youngs_poisson(E_stretch, nu_stretch));
    auto lambda2 = mesh2.triangles().find<Float>("lambda");
    auto sr2     = mesh2.triangles().find<Float>("strain_rate");
    REQUIRE(lambda2);
    REQUIRE(sr2);
    CHECK(std::ranges::all_of(
        lambda2->view(),
        [&](Float v) { return v == Catch::Approx(expect_stretch); }));
    CHECK(std::ranges::all_of(
        sr2->view(), [](Float v) { return v == Catch::Approx(100.0); }));
}
