#include <catch.hpp>
#include <uipc/geometry/factory.h>
#include <spdlog/spdlog.h>
#include <numeric>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("shared_test", "[geometry]")
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto mesh = tetmesh(Vs, Ts);

    auto shared_mesh = mesh;
    REQUIRE(shared_mesh.positions().is_shared());

    Ts.size();

    // a const view just references the data
    auto const_view = std::as_const(shared_mesh).positions().view();

    REQUIRE(shared_mesh.positions().is_shared());

    // a non-const view creates a clone of the data if it is not owned
    auto non_const_view = shared_mesh.positions().view();

    // after that, the data is owned
    REQUIRE(!shared_mesh.positions().is_shared());


    auto VA  = shared_mesh.vertices();
    auto pos = VA.find<Vector3>("position");
    REQUIRE(pos->size() == Vs.size());
    REQUIRE(!pos->is_shared());

    auto TA = shared_mesh.tetrahedra();
    // These query don't modify the data, so the data is not cloned
    REQUIRE(VA.size() == Vs.size());
    REQUIRE(TA.size() == Ts.size());
    REQUIRE(VA.topo_is_shared());
    REQUIRE(TA.topo_is_shared());


    // when resize:
    //  - the vertex topo is owned
    //  - because the 'position' attribute is already owned, it is not cloned
    VA.resize(8);
    auto pos_view = pos->view();
    pos_view[4]   = Vector3{1.0, 1.0, 0.0};
    pos_view[5]   = Vector3{1.0, 0.0, 1.0};
    pos_view[6]   = Vector3{0.0, 1.0, 1.0};
    pos_view[7]   = Vector3{1.0, 1.0, 1.0};

    Vector3 center =
        std::accumulate(pos_view.begin(), pos_view.end(), Vector3{0, 0, 0}) / pos->size();

    // a clone is made here
    TA.resize(2);
    auto tet_view = TA.topo_view();
    tet_view[1]   = Vector4i{0, 1, 3, 5};

    REQUIRE(pos->size() == 8);
    REQUIRE(VA.size() == 8);
    REQUIRE(TA.size() == 2);

    // after changing the topo, the topo is owned
    REQUIRE(!VA.topo_is_shared());
    REQUIRE(!TA.topo_is_shared());

    // shallow copy, the data is not cloned
    auto shared_topo_mesh = shared_mesh;

    // Here we want share the topo, but own the attributes.
    {
        // make a non const view, the data is automatically cloned
        auto view = shared_topo_mesh.positions().view();
        // so the positions are owned
        REQUIRE(!shared_topo_mesh.positions().is_shared());
        // but the topo are shared
        REQUIRE(shared_topo_mesh.vertices().topo_is_shared());
    }
}


SimplicialComplex create_tetrahedron()
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    return tetmesh(Vs, Ts);
}


TEST_CASE("create_attribute", "[geometry]")
{
    auto mesh = create_tetrahedron();

    auto VA = mesh.vertices();

    auto& vel = VA.create<Vector3>("velocity");
    REQUIRE(vel.size() == VA.size());


    REQUIRE_THROWS_AS(VA.create<Vector3>("velocity"), AttributeAlreadyExist);
}
