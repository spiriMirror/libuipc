#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <filesystem>
#include <fstream>

TEST_CASE("init_surface_intersection", "[init_surface]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"none", this_output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = true;
    Scene scene{config};

    // create object
    auto object = scene.objects().create("tets");

    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    {
        vector<Vector3> Vs = {Vector3{0, 1, 0},
                              Vector3{0, 0, 1},
                              Vector3{-std::sqrt(3) / 2, 0, -0.5},
                              Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.3; });

        auto mesh1 = tetmesh(Vs, Ts);

        label_surface(mesh1);
        label_triangle_orient(mesh1);

        auto trans_view = view(mesh1.transforms());
        auto is_fixed   = mesh1.instances().find<IndexT>(builtin::is_fixed);

        {
            Transform t     = Transform::Identity();
            t.translation() = Vector3::UnitY() * 0.15;
            trans_view[0]   = t.matrix();
        }

        object->geometries().create(mesh1);
    }

    {
        vector<Vector3> Vs = {Vector3{0, 1, 0},
                              Vector3{0, 0, 1},
                              Vector3{-std::sqrt(3) / 2, 0, -0.5},
                              Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.3; });


        auto mesh2 = tetmesh(Vs, Ts);

        label_surface(mesh2);
        label_triangle_orient(mesh2);

        auto trans_view = view(mesh2.transforms());

        {
            Transform t   = Transform::Identity();
            trans_view[0] = t.matrix();
        }

        object->geometries().create(mesh2);
    }

    world.init(scene);

    REQUIRE(!world.is_valid());
}
