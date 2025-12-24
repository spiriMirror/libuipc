#include <app/catch2.h>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <filesystem>
#include <fstream>

TEST_CASE("45_abd_prismatic_joint", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs          = std::filesystem;
    auto this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                            = Scene::default_config();
    config["gravity"]                      = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"]            = true;
    config["line_search"]["report_energy"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};

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
        // Create 2 instances for left geometry
        left_mesh.instances().resize(2);
        Transform t0 = Transform::Identity();
        t0.translate(Vector3::UnitX() * -0.6 + Vector3::UnitZ() * -0.5);
        view(left_mesh.transforms())[0] = t0.matrix();

        Transform t1 = Transform::Identity();
        t1.translate(Vector3::UnitX() * -0.6 + Vector3::UnitZ() * 0.5);
        view(left_mesh.transforms())[1] = t1.matrix();

        auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 0;  // free instance 0
        view(*is_fixed)[1] = 1;  // fix instance 1
    }
    auto [left_geo_slot, left_rest_geo_slot] = left_link->geometries().create(left_mesh);

    SimplicialComplex right_mesh = abd_mesh;
    {
        // Create 2 instances for right geometry
        right_mesh.instances().resize(2);
        Transform t0 = Transform::Identity();
        t0.translate(Vector3::UnitX() * 0.6 + Vector3::UnitZ() * -0.5);
        view(right_mesh.transforms())[0] = t0.matrix();

        Transform t1 = Transform::Identity();
        t1.translate(Vector3::UnitX() * 0.6 + Vector3::UnitZ() * 0.5);
        view(right_mesh.transforms())[1] = t1.matrix();

        auto is_fixed = right_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;  // fix instance 0
        view(*is_fixed)[1] = 0;  // free instance 1
    }
    auto [right_geo_slot, right_rest_geo_slot] = right_link->geometries().create(right_mesh);

    // Each edge defines a joint axis (2 points per edge for prismatic joint)
    vector<Vector2i> Es = {{0, 1}, {2, 3}};

    vector<Vector3> Vs = {Vector3{0.5, -0.5, -0.5},
                          Vector3{-0.5, 0.5, -0.5},
                          Vector3{0.5, 0.5, 0.5},
                          Vector3{-0.5, -0.5, 0.5}};

    auto joint_mesh = linemesh(Vs, Es);

    label_surface(joint_mesh);  // visualize the joint
    AffineBodyPrismaticJoint prismatic_joint;

    // Use multi-instance API:
    vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot, left_geo_slot};
    vector<IndexT> l_instance_id = {0, 1};
    vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot, right_geo_slot};
    vector<IndexT> r_instance_id   = {0, 1};
    vector<Float>  strength_ratios = {100.0, 100.0};

    prismatic_joint.apply_to(joint_mesh,
                             span{l_geo_slots},
                             span{l_instance_id},
                             span{r_geo_slots},
                             span{r_instance_id},
                             span{strength_ratios});
    auto joints = scene.objects().create("prismatic_joint");
    joints->geometries().create(joint_mesh);

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