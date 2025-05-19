#include <app/test_common.h>
#include <uipc/uipc.h>
#include <numeric>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("shared_attribute", "[simplicial_complex]")
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto mesh = tetmesh(Vs, Ts);

    auto shared_mesh = mesh;
    REQUIRE(shared_mesh.positions().is_shared());

    // a const view just references the data
    auto const_view = shared_mesh.positions().view();

    REQUIRE(shared_mesh.positions().is_shared());

    // a non-const view creates a clone of the data if it is not owned
    auto non_const_view = view(shared_mesh.positions());

    // after that, the data is owned
    REQUIRE(!shared_mesh.positions().is_shared());


    auto VA  = shared_mesh.vertices();
    auto pos = VA.find<Vector3>(builtin::position);
    REQUIRE(pos->size() == Vs.size());
    REQUIRE(!pos->is_shared());

    auto TA = shared_mesh.tetrahedra();
    // These query don't modify the data, so the data is not cloned
    REQUIRE(VA.size() == Vs.size());
    REQUIRE(TA.size() == Ts.size());
    REQUIRE(TA.topo().is_shared());


    // when resize:
    //  - the vertex topo is owned
    //  - because the 'position' attribute is already owned, it is not cloned
    VA.resize(8);
    auto pos_view = view(*pos);
    pos_view[4]   = Vector3{1.0, 1.0, 0.0};
    pos_view[5]   = Vector3{1.0, 0.0, 1.0};
    pos_view[6]   = Vector3{0.0, 1.0, 1.0};
    pos_view[7]   = Vector3{1.0, 1.0, 1.0};

    Vector3 center =
        std::accumulate(pos_view.begin(), pos_view.end(), Vector3{0, 0, 0}) / pos->size();

    // a clone is made here
    TA.resize(2);
    auto tet_view = view(TA.topo());

    tet_view[1] = Vector4i{0, 1, 3, 5};

    REQUIRE(pos->size() == 8);
    REQUIRE(VA.size() == 8);
    REQUIRE(TA.size() == 2);

    // after changing the topo, the topo is owned
    REQUIRE(!TA.topo().is_shared());

    // shallow copy_from, the data is not cloned
    auto shared_topo_mesh = shared_mesh;

    // Here we want share the topo, but own the attributes.
    {
        // make a non const view, the data is automatically cloned
        auto view = geometry::view(shared_topo_mesh.positions());
        // so the positions are owned
        REQUIRE(!shared_topo_mesh.positions().is_shared());
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

SimplicialComplex create_point_cloud()
{
    std::vector Vs = {Vector3{0.0, 0.0, 0.0}};

    return pointcloud(Vs);
}


TEST_CASE("create_delete_share_attribute", "[simplicial_complex]")
{
    auto mesh = create_tetrahedron();

    auto point_cloud = create_point_cloud();

    auto& pos = mesh.positions();

    auto VA = mesh.vertices();

    auto vel = VA.create<Vector3>("velocity", Vector3::Zero());
    REQUIRE(vel->size() == VA.size());


    REQUIRE_THROWS_AS(VA.create<Vector3>("velocity"), AttributeCollectionError);

    VA.destroy("velocity");
    REQUIRE_ONCE_WARN(VA.destroy("velocity"));

    auto find_vel = VA.find<Vector3>("velocity");
    REQUIRE(!find_vel);
    REQUIRE(find_vel.use_count() == 0);

    REQUIRE_THROWS_AS(VA.destroy(builtin::position), AttributeCollectionError);

    VA.share("velocity", pos);
    REQUIRE(VA.find<Vector3>("velocity")->is_shared());

    REQUIRE_THROWS_AS(VA.share("velocity", point_cloud.positions()), AttributeCollectionError);
}

TEST_CASE("const_attribute", "[simplicial_complex]")
{
    auto mesh = create_tetrahedron();

    const auto& const_mesh = mesh;

    auto VA  = mesh.vertices();
    auto CVA = const_mesh.vertices();

    REQUIRE(CVA.size() == VA.size());
    REQUIRE(CVA.find<Vector3>(builtin::position));

    auto TA  = mesh.tetrahedra();
    auto CTA = const_mesh.tetrahedra();

    REQUIRE(TA.size() == CTA.size());
    REQUIRE(!CTA.find<U64>("FAKE"));
    REQUIRE(!CTA.topo().is_shared());

    REQUIRE(std::ranges::equal(TA.topo().view(), CTA.topo().view()));
}

TEST_CASE("print", "[simplicial_complex]")
{
    auto mesh = create_tetrahedron();

    fmt::println("print simplicial complex:");
    fmt::println(R"({{
{}
}})",
                 mesh);

    fmt::println("");
    fmt::println("print attributes:");
    fmt::println("meta: {}", mesh.meta());
    fmt::println("instances:{}", mesh.instances());
    fmt::println("vertices:{}", mesh.vertices());
    fmt::println("edges:{}", mesh.edges());
    fmt::println("triangles:{}", mesh.triangles());
    fmt::println("tetrahedra:{}", mesh.tetrahedra());
    fmt::println("");
}

TEST_CASE("utils", "[simplicial_complex]")
{
    using SU = SimplexUtils;
    REQUIRE(SU::is_same_edge(Vector2i{0, 10}, Vector2i{0, 11}) == false);
    REQUIRE(SU::is_same_edge(Vector2i{0, 10}, Vector2i{10, 0}) == true);
}

TEST_CASE("geometry_commit", "[simplicial_complex]")
{
    auto mesh = create_tetrahedron();

    SimplicialComplex mesh_copy = mesh;

    auto name = mesh.meta().create<std::string>("name");
    // update
    auto pos_view = view(mesh.positions());
    std::ranges::transform(pos_view,
                           pos_view.begin(),
                           [](auto& p) { return p + Vector3{1.0, 1.0, 1.0}; });

    REQUIRE(mesh_copy.positions().last_modified() < mesh.positions().last_modified());

    GeometryCommit commit = mesh - mesh_copy;
    mesh_copy.update_from(commit);

    auto name_copy = mesh_copy.meta().find<std::string>("name");
    REQUIRE(name_copy != nullptr);

    auto dst_pos_view = view(mesh_copy.positions());
    auto src_pos_view = view(mesh.positions());
    REQUIRE(std::ranges::equal(src_pos_view, dst_pos_view));
}