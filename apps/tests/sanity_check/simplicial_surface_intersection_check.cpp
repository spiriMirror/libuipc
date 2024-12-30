#include <catch.hpp>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <filesystem>
#include <fstream>

void test_init_surf_intersection_check(std::string_view            name,
                                       std::string_view            mesh,
                                       uipc::span<uipc::Transform> trans)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path =
        AssetDir::output_path(__FILE__) + fmt::format("/{}-{}", name, trans.size());

    Engine engine{"none", this_output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = true;
    // config["sanity_check"]["mode"]   = "quiet";
    Scene scene{config};

    // create object
    auto object = scene.objects().create("meshes");

    for(auto& t : trans)
    {
        SimplicialComplexIO io{t};
        auto                m = io.read(fmt::format("{}", mesh));
        label_surface(m);

        if(m.dim() == 3)
            label_triangle_orient(m);
        object->geometries().create(m);
    }


    if(trans.size() == 1)
    {
        world.init(scene);
        REQUIRE(world.is_valid());
    }
    else
    {
        REQUIRE_HAS_ERROR(world.init(scene));
        REQUIRE(!world.is_valid());

        auto& msg  = scene.sanity_checker().errors().at(1);
        auto& mesh = msg->geometries().at("intersected_mesh");
        auto  sc   = mesh->as<SimplicialComplex>();
        REQUIRE(sc->edges().size() > 0);
        REQUIRE(sc->triangles().size() > 0);
    }
}

TEST_CASE("simplicial_surface_intersection", "[init_surface]")
{
    using namespace uipc;
    auto tetmesh_dir = AssetDir::tetmesh_path();
    auto trimesh_dir = AssetDir::trimesh_path();

    {
        auto              name = "cube.obj";
        auto              path = fmt::format("{}/{}", trimesh_dir, name);
        vector<Transform> transforms;
        for(auto i = 0; i < 2; ++i)
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.2 * i);
            transforms.push_back(t);
        }
        test_init_surf_intersection_check(name, path, span{transforms});
        test_init_surf_intersection_check(name, path, span{transforms}.subspan<0, 1>());
    }

    {
        auto              name = "bunny0.msh";
        auto              path = fmt::format("{}/{}", tetmesh_dir, name);
        vector<Transform> transforms;
        for(auto i = 0; i < 2; ++i)
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.2 * i);
            transforms.push_back(t);
        }
        test_init_surf_intersection_check(name, path, span{transforms});
        test_init_surf_intersection_check(name, path, span{transforms}.subspan<0, 1>());
    }

    {
        auto              name = "link.msh";
        auto              path = fmt::format("{}/{}", tetmesh_dir, name);
        vector<Transform> transforms;
        for(auto i = 0; i < 2; ++i)
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.2 * i);
            transforms.push_back(t);
        }
        test_init_surf_intersection_check(name, path, span{transforms});
        test_init_surf_intersection_check(name, path, span{transforms}.subspan<0, 1>());
    }

    {
        auto              name = "ball.msh";
        auto              path = fmt::format("{}/{}", tetmesh_dir, name);
        vector<Transform> transforms;
        for(auto i = 0; i < 2; ++i)
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.2 * i);
            transforms.push_back(t);
        }
        test_init_surf_intersection_check(name, path, span{transforms});
        test_init_surf_intersection_check(name, path, span{transforms}.subspan<0, 1>());
    }
}
