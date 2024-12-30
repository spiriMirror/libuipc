#include <catch.hpp>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <filesystem>
#include <fstream>


void test_half_plane_vertex_distance_check(std::string_view name, std::string_view mesh)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto this_output_path = AssetDir::output_path(__FILE__) + fmt::format("/{}", name);

    Engine engine{"none", this_output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = true;
    config["sanity_check"]["mode"]   = "quiet";
    Scene scene{config};

    // create object
    auto object = scene.objects().create("meshes");

    SimplicialComplexIO io;
    auto                m = io.read(fmt::format("{}", mesh));
    label_surface(m);

    if(m.dim() == 3)
        label_triangle_orient(m);
    object->geometries().create(m);

    auto Vs = m.positions().view();

    // calculate the mid point
    Vector3 mid = Vector3::Zero();
    for(auto& v : Vs)
        mid += v;
    mid /= Vs.size();

    // create a half plane
    auto g = ground(mid.y());
    object->geometries().create(g);

    // create a x+ half plane
    auto g1 = halfplane(mid, Vector3::UnitX());
    object->geometries().create(g1);

    REQUIRE_HAS_ERROR(world.init(scene));
    REQUIRE(!world.is_valid());

    {
        auto& msg  = scene.sanity_checker().errors().at(2);
        auto& mesh = msg->geometries().at("close_mesh");

        SimplicialComplexIO io;
        io.write(fmt::format("{}/{}.obj", this_output_path, name),
                 *mesh->as<SimplicialComplex>());
    }
}


TEST_CASE("half_plane_vertex_distance", "[init_surface]")
{
    using namespace uipc;
    auto tetmesh_dir = AssetDir::tetmesh_path();
    auto trimesh_dir = AssetDir::trimesh_path();

    {
        auto name = "cube.obj";
        auto path = fmt::format("{}/{}", trimesh_dir, name);
        test_half_plane_vertex_distance_check(name, path);
    }

    {
        auto name = "tet.msh";
        auto path = fmt::format("{}/{}", tetmesh_dir, name);
        test_half_plane_vertex_distance_check(name, path);
    }

    {
        auto name = "bunny0.msh";
        auto path = fmt::format("{}/{}", tetmesh_dir, name);
        test_half_plane_vertex_distance_check(name, path);
    }

    {
        auto name = "link.msh";
        auto path = fmt::format("{}/{}", tetmesh_dir, name);
        test_half_plane_vertex_distance_check(name, path);
    }

    {
        auto name = "ball.msh";
        auto path = fmt::format("{}/{}", tetmesh_dir, name);
        test_half_plane_vertex_distance_check(name, path);
    }
}