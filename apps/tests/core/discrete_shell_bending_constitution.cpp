#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/plastic_discrete_shell_bending.h>
#include <algorithm>

TEST_CASE("discrete_shell_bending_attribute_layout", "[constitution]")
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

    auto elastic_mesh = trimesh(Vs, Fs);
    auto plastic_mesh = trimesh(Vs, Fs);

    DiscreteShellBending        dsb;
    PlasticDiscreteShellBending pdsb;

    dsb.apply_to(elastic_mesh, 5.0_kPa);
    pdsb.apply_to(plastic_mesh, 5.0_kPa, 0.05, 0.0);

    auto elastic_bending = elastic_mesh.edges().find<Float>("bending_stiffness");
    REQUIRE(elastic_bending);
    CHECK(elastic_bending->size() == elastic_mesh.edges().size());
    CHECK(std::ranges::all_of(
        elastic_bending->view(), [](Float v) { return v == Catch::Approx(5.0_kPa); }));
    CHECK(elastic_mesh.edges().find<Float>("bending_yield_threshold") == nullptr);
    CHECK(elastic_mesh.edges().find<Float>("bending_hardening_modulus") == nullptr);

    auto plastic_bending = plastic_mesh.edges().find<Float>("bending_stiffness");
    auto plastic_yield   = plastic_mesh.edges().find<Float>("bending_yield_threshold");
    auto plastic_hardening =
        plastic_mesh.edges().find<Float>("bending_hardening_modulus");

    REQUIRE(plastic_bending);
    REQUIRE(plastic_yield);
    REQUIRE(plastic_hardening);
    CHECK(plastic_bending->size() == plastic_mesh.edges().size());
    CHECK(plastic_yield->size() == plastic_mesh.edges().size());
    CHECK(plastic_hardening->size() == plastic_mesh.edges().size());
    CHECK(std::ranges::all_of(
        plastic_bending->view(), [](Float v) { return v == Catch::Approx(5.0_kPa); }));
    CHECK(std::ranges::all_of(
        plastic_yield->view(), [](Float v) { return v == Catch::Approx(0.05); }));
    CHECK(std::ranges::all_of(
        plastic_hardening->view(), [](Float v) { return v == Catch::Approx(0.0); }));
}
