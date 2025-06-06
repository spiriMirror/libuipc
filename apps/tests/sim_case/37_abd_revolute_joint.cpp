#include <catch.hpp>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <filesystem>
#include <fstream>

TEST_CASE("37_abd_revolute_joint", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    auto this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                 = Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        scene.contact_tabular().default_model(0.5, 1.0_GPa);

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.4);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto left_link  = scene.objects().create("left");
        auto right_link = scene.objects().create("right");

        AffineBodyConstitution abd;
        SimplicialComplex      abd_mesh =
            io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        abd.apply_to(abd_mesh, 100.0_MPa);
        label_surface(abd_mesh);

        SimplicialComplex left_mesh = abd_mesh;
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitX() * -0.6);
            view(left_mesh.transforms())[0] = t.matrix();
            auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
            // view(*is_fixed)[0] = 0;  // fix the right link
        }
        auto [left_geo_slot, left_rest_geo_slot] =
            left_link->geometries().create(left_mesh);

        SimplicialComplex right_mesh = abd_mesh;
        {
            Transform t = Transform::Identity();
            t.translate(Vector3::UnitX() * 0.6);
            view(right_mesh.transforms())[0] = t.matrix();
            auto is_fixed = right_mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;  // fix the right link
        }
        auto [right_geo_slot, right_rest_geo_slot] =
            right_link->geometries().create(right_mesh);

        AffineBodyRevoluteJoint abrj;

        using SlotTuple = AffineBodyRevoluteJoint::SlotTuple;
        vector<SlotTuple> links;

        links.push_back({left_geo_slot, right_geo_slot});

        vector<Vector2i> Es         = {{0, 1}};
        vector<Vector3>  Vs         = {{0, 0, -1.0}, {0, 0, 1.0}};
        auto             joint_mesh = linemesh(Vs, Es);
        label_surface(joint_mesh);  // visualize the joint

        abrj.apply_to(joint_mesh, links);
        auto joints = scene.objects().create("joint");
        joints->geometries().create(joint_mesh);
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(
        fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    REQUIRE(world.is_valid());


    while(world.frame() < 100)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}