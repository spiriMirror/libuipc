#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/backend/visitors/scene_visitor.h>


TEST_CASE("scene", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene scene;

    auto object = scene.objects().create("cube");

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto [geo, rest_geo] = object->geometries().create(mesh);

    const auto& const_obj = *object;

    auto ID = object->id();
    scene.objects().destroy(ID);

    REQUIRE_ONCE_WARN(scene.objects().destroy(ID));

    object = scene.objects().create("cube1");

    // create two geometries
    object->geometries().create(mesh);
    object->geometries().create(mesh);

    REQUIRE(object->geometries().ids().size() == 2);
    REQUIRE(object->geometries().ids()[0] == 1);
    REQUIRE(object->geometries().ids()[1] == 2);
}


TEST_CASE("pending sequence", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene scene;
    auto  object = scene.objects().create("cube");
    REQUIRE(object->id() == 0);

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto [geo, rest_geo] = object->geometries().create(mesh);

    backend::SceneVisitor visitor{scene};
    visitor.begin_pending();

    auto new_object = scene.objects().create("cube2");
    new_object->geometries().create(mesh);
    REQUIRE(new_object->id() == 1);


    // only geometries will be added to pending list

    // object is not added to pending list
    // so the creation of object is immediate
    REQUIRE(scene.objects().size() == 2);

    // the geometries are added to pending list
    // so the creation of geometries is delayed
    REQUIRE(visitor.geometries().size() == 1);
    REQUIRE(visitor.pending_geometries().size() == 1);
    REQUIRE(visitor.rest_geometries().size() == 1);
    REQUIRE(visitor.pending_rest_geometries().size() == 1);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    // compute_contact the pending list
    visitor.solve_pending();

    // all the pending geometries are created
    REQUIRE(visitor.geometries().size() == 2);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 2);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    // destroy the object
    scene.objects().destroy(new_object->id());

    // only geometries will be added to pending list

    // object is not added to pending list
    // so the destruction of object is immediate
    REQUIRE(scene.objects().size() == 1);

    // the geometries are added to pending list
    // so the destruction of geometries is delayed
    REQUIRE(visitor.geometries().size() == 2);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 2);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 1);

    // compute_contact the pending list
    visitor.solve_pending();

    // all the pending geometries are destroyed
    REQUIRE(visitor.geometries().size() == 1);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.rest_geometries().size() == 1);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    // create a new object immediately
    new_object = scene.objects().create("cube3");

    REQUIRE(new_object->id() == 2);
    REQUIRE(scene.objects().size() == 2);
    
    // create 3 geometries in pending list
    new_object->geometries().create(mesh);
    new_object->geometries().create(mesh);
    new_object->geometries().create(mesh);

    REQUIRE(visitor.pending_geometries().size() == 3);
    REQUIRE(visitor.pending_rest_geometries().size() == 3);

    // destroy the object
    REQUIRE_ALL_INFO(scene.objects().destroy(new_object->id()));

    // so the geometries are first added to pending list
    // and then removed from pending list
    // so we can't see the geometries in pending create/destroy list
    REQUIRE(visitor.pending_destroy_ids().size() == 0);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
}

TEST_CASE("pending create", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene                 scene;
    backend::SceneVisitor visitor{scene};

    auto object = scene.objects().create("cube");
    REQUIRE(object->id() == 0);

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto [geo, rest_geo] = object->geometries().create(mesh);

    // because now we don't start pending, so the geometries are created immediately
    {
        REQUIRE(visitor.geometries().size() == 1);
        REQUIRE(visitor.rest_geometries().size() == 1);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }

    visitor.begin_pending();

    // now we start pending, so the geometries are added to pending list
    {
        auto new_object = scene.objects().create("cube2");
        REQUIRE(scene.objects().size() == 2);

        new_object->geometries().create(mesh);

        new_object = scene.objects().create("cube3");
        REQUIRE(scene.objects().size() == 3);

        new_object->geometries().create(mesh);
        new_object->geometries().create(mesh);
        new_object->geometries().create(mesh);

        REQUIRE(visitor.pending_geometries().size() == 4);
        REQUIRE(visitor.pending_rest_geometries().size() == 4);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }

    visitor.solve_pending();

    {
        REQUIRE(visitor.geometries().size() == 5);
        REQUIRE(visitor.rest_geometries().size() == 5);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
    }
}

TEST_CASE("pending destroy", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    Scene                 scene;
    backend::SceneVisitor visitor{scene};

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));


    constexpr int N = 5;

    vector<IndexT> ids;
    vector<IndexT> geo_ids;

    for(int i = 0; i < N; i++)
    {
        auto new_object = scene.objects().create("cube2");
        new_object->geometries().create(mesh);
        ids.push_back(new_object->id());
        for(auto id : new_object->geometries().ids())
        {
            geo_ids.push_back(id);
        }
    }
    REQUIRE(scene.objects().size() == N);
    REQUIRE(visitor.geometries().size() == N);
    REQUIRE(visitor.rest_geometries().size() == N);
    REQUIRE(visitor.pending_geometries().size() == 0);
    REQUIRE(visitor.pending_rest_geometries().size() == 0);
    REQUIRE(visitor.pending_destroy_ids().size() == 0);

    visitor.begin_pending();

    {
        for(auto id : ids)
        {
            scene.objects().destroy(id);
        }

        // immediately
        REQUIRE(scene.objects().size() == 0);

        // pending
        REQUIRE(visitor.geometries().size() == N);
        REQUIRE(visitor.rest_geometries().size() == N);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
        REQUIRE(visitor.pending_destroy_ids().size() == N);

        for(auto id : geo_ids)
        {
            auto&& [geo, rest_geo] = scene.geometries().find(id);
        }
    }

    visitor.solve_pending();

    {
        REQUIRE(visitor.pending_destroy_ids().size() == 0);
        REQUIRE(visitor.geometries().size() == 0);
        REQUIRE(visitor.rest_geometries().size() == 0);
        REQUIRE(visitor.pending_geometries().size() == 0);
        REQUIRE(visitor.pending_rest_geometries().size() == 0);
    }
}
