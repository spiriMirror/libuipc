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
    Float thickness   = 0.001_m;
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

    // Stored thickness is one-sided; stretch uses the full shell thickness 2r.
    Float expect_stretch = E_stretch * (2 * thickness) / (1.0 - nu_stretch * nu_stretch);
    // shear attr = E_shear / (2 * (1 + nu_shear))  (thickness-independent)
    Float expect_shear = E_shear / (2.0 * (1.0 + nu_shear));

    CHECK(std::ranges::all_of(lambda->view(),
                              [&](Float v)
                              { return v == Catch::Approx(expect_stretch); }));
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
    CHECK(std::ranges::all_of(lambda2->view(),
                              [&](Float v)
                              { return v == Catch::Approx(expect_stretch); }));
    CHECK(std::ranges::all_of(sr2->view(),
                              [](Float v) { return v == Catch::Approx(100.0); }));
}

TEST_CASE("strain_limiting_baraff_witkin_directional_moduli", "[constitution][baraff_witkin]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    const vector<Vector3> vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const vector<Vector3i>          triangles = {{0, 1, 2}, {0, 2, 3}};
    auto                            mesh      = trimesh(vertices, triangles);
    StrainLimitingBaraffWitkinShell material;
    auto            a      = ElasticModuli2D::youngs_poisson(2e4, 0.3);
    auto            b      = ElasticModuli2D::youngs_poisson(8e3, 0.2);
    auto            shear  = ElasticModuli2D::youngs_poisson(10, 0.25);
    constexpr Float radius = 0.002;
    const Float     ka     = 2e4 * (2 * radius) / (1 - 0.3 * 0.3);
    const Float     kb     = 8e3 * (2 * radius) / (1 - 0.2 * 0.2);
    const Float     kg     = 10 / (2 * (1 + 0.25));

    auto check_values = [&](std::string_view name, Float expected)
    {
        auto attr = mesh.triangles().find<Float>(name);
        REQUIRE(attr);
        REQUIRE(attr->size() == 2);
        for(Float value : attr->view())
            CHECK(value == Catch::Approx(expected));
    };

    // Legacy layout stays shared even after a per-triangle lambda edit.
    material.apply_to(mesh, a, shear, 200, radius, 40);
    CHECK_FALSE(mesh.triangles().find<Float>("stretch_stiffness_b"));
    view(*mesh.triangles().find<Float>("lambda"))[0] = 123;
    CHECK_FALSE(mesh.triangles().find<Float>("stretch_stiffness_b"));

    material.apply_to(mesh, a, b, shear, 200, radius, 40);
    check_values("lambda", ka);
    check_values("stretch_stiffness_b", kb);
    check_values("mu", kg);
    check_values("strain_rate", 40);

    // Reapplication overwrites every triangle, including prior manual edits.
    view(*mesh.triangles().find<Float>("stretch_stiffness_b"))[1] = 321;
    material.apply_to(mesh, b, a, shear, 200, radius, 20);
    check_values("lambda", kb);
    check_values("stretch_stiffness_b", ka);
    check_values("mu", kg);
    check_values("strain_rate", 20);

    material.apply_to(mesh, a, shear, 200, radius, 40);
    CHECK_FALSE(mesh.triangles().find<Float>("stretch_stiffness_b"));
    check_values("lambda", ka);
    check_values("mu", kg);
    material.apply_to(mesh, a, b, shear);
    material.apply_to(mesh, b);
    CHECK_FALSE(mesh.triangles().find<Float>("stretch_stiffness_b"));
    check_values("lambda", 8e3 * 0.002 / (1 - 0.2 * 0.2));

    // Generic scene serialization retains independent coefficients, and legacy
    // scenes retain absence of the override instead of silently adding one.
    for(bool independent : {false, true})
    {
        if(independent)
            material.apply_to(mesh, a, b, shear, 200, radius, 40);
        core::Scene scene;
        auto        object     = scene.objects().create("cloth");
        auto [slot, rest_slot] = object->geometries().create(mesh);
        auto loaded = core::SceneIO::from_json(core::SceneIO{scene}.to_json());
        auto [loaded_slot, loaded_rest] = loaded.geometries().find(slot->id());
        const auto& restored = loaded_slot->geometry().as<SimplicialComplex>();
        REQUIRE(restored);
        auto restored_b = restored->triangles().find<Float>("stretch_stiffness_b");
        CHECK(static_cast<bool>(restored_b) == independent);
        if(independent)
        {
            for(Float value : restored_b->view())
                CHECK(value == Catch::Approx(kb));
        }
    }
}
