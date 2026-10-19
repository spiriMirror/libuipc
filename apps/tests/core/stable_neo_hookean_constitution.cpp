#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

TEST_CASE("stable_neo_hookean_material_conversion", "[constitution][stable_neo_hookean]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    const vector<Vector3> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
    const vector<Vector4i> cells = {{0, 1, 2, 3}, {1, 2, 3, 4}};
    auto                   mesh  = tetmesh(vertices, cells);
    StableNeoHookean       material;
    material.apply_to(mesh);
    auto mus     = mesh.tetrahedra().find<Float>("mu");
    auto lambdas = mesh.tetrahedra().find<Float>("lambda");
    REQUIRE(mus);
    REQUIRE(lambdas);
    REQUIRE(mus->size() == cells.size());
    REQUIRE(lambdas->size() == cells.size());
    const auto defaults = ElasticModuli::youngs_poisson(120.0_kPa, 0.49);
    CHECK(mus->view().front() == Catch::Approx(defaults.mu()));
    CHECK(lambdas->view().front() == Catch::Approx(defaults.lambda() + defaults.mu()));

    // Reapply through each public parameterization; every cell must be updated.
    for(Float E : {1.0e3, 1.0e5, 1.0e8})
    {
        for(Float nu : {-0.2, 0.0, 0.3, 0.49, 0.499})
        {
            CAPTURE(E, nu);
            const Float mu     = E / (2 * (1 + nu));
            const Float lambda = E * nu / ((1 + nu) * (1 - 2 * nu));
            for(const auto& moduli : {ElasticModuli::youngs_poisson(E, nu),
                                      ElasticModuli::youngs_shear(E, mu),
                                      ElasticModuli::lame(lambda, mu)})
            {
                material.apply_to(mesh, moduli);
                for(SizeT i = 0; i < cells.size(); ++i)
                {
                    CHECK(mus->view()[i] == Catch::Approx(mu));
                    CHECK(lambdas->view()[i] == Catch::Approx(lambda + mu));
                }
            }
        }
    }
}
