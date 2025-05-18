#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

TEST_CASE("scene_io", "[scene]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    Scene scene;
    auto  object = scene.objects().create("objects");

    SimplicialComplexIO io;
    auto cube_mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    label_surface(cube_mesh);
    label_triangle_orient(cube_mesh);

    auto tet_mesh = io.read(fmt::format("{}tet.msh", AssetDir::tetmesh_path()));
    label_surface(tet_mesh);
    label_triangle_orient(tet_mesh);


    cube_mesh.instances().resize(2);

    Transform t = Transform::Identity();

    for(int i = 0; i < 2; i++)
    {
        t.translate(Vector3::UnitY() * 1.5);
        auto Trans = view(cube_mesh.transforms());
        Trans[i]   = t.matrix();
    }

    tet_mesh.instances().resize(3);
    for(int i = 0; i < 3; i++)
    {
        t.translate(Vector3::UnitY() * 1.5);
        auto Trans = view(tet_mesh.transforms());
        Trans[i]   = t.matrix();
    }

    object->geometries().create(cube_mesh);
    object->geometries().create(tet_mesh);


    SceneIO scene_io{scene};

    scene_io.write_surface(fmt::format("{}scene.obj", AssetDir::output_path(__FILE__)));
    scene_io.save(fmt::format("{}scene.json", AssetDir::output_path(__FILE__)));

    auto scene_loaded =
        SceneIO::load(fmt::format("{}scene.json", AssetDir::output_path(__FILE__)));

    auto object_loaded = scene_loaded.objects().find(0);
    REQUIRE(object_loaded->name() == object->name());
    auto objects_found = scene_loaded.objects().find("objects");
    REQUIRE(objects_found.size() == 1);
}
