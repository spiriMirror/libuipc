#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>

TEST_CASE("37_abd_revolute_joint", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;
    test::Scene::dump_config(config, output_path);

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
        abd_mesh.instances().resize(2);
        label_surface(abd_mesh);
        abd.apply_to(abd_mesh, 100.0_MPa);

        SimplicialComplex left_mesh = abd_mesh;
        {
            // Create 2 instances for left geometry
            Transform t0 = Transform::Identity();
            t0.translate(Vector3::UnitX() * -0.6 + Vector3::UnitZ() * -0.5);
            view(left_mesh.transforms())[0] = t0.matrix();

            Transform t1 = Transform::Identity();
            t1.translate(Vector3::UnitX() * -0.6 + Vector3::UnitZ() * 0.5);
            view(left_mesh.transforms())[1] = t1.matrix();

            auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 0;  // instance 0 not fixed
            view(*is_fixed)[1] = 0;  // instance 1 not fixed
        }
        auto [left_geo_slot, left_rest_geo_slot] =
            left_link->geometries().create(left_mesh);

        SimplicialComplex right_mesh = abd_mesh;
        {
            // Create 2 instances for right geometry
            Transform t0 = Transform::Identity();
            t0.translate(Vector3::UnitX() * 0.6 + Vector3::UnitZ() * -0.5);
            view(right_mesh.transforms())[0] = t0.matrix();

            Transform t1 = Transform::Identity();
            t1.translate(Vector3::UnitX() * 0.6 + Vector3::UnitZ() * 0.5);
            view(right_mesh.transforms())[1] = t1.matrix();

            auto is_fixed = right_mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;  // fix instance 0
            view(*is_fixed)[1] = 1;  // fix instance 1
        }
        auto [right_geo_slot, right_rest_geo_slot] =
            right_link->geometries().create(right_mesh);

        AffineBodyRevoluteJoint abrj;

        // Each edge defines a joint axis (2 points per edge)
        vector<Vector2i> Es = {{0, 1}, {2, 3}};
        vector<Vector3>  Vs = {{0, 0, 0.5 - 0.5},   //
                               {0, 0, 0.5 + 0.5},   //
                               {0, 0, -0.5 - 0.5},  //
                               {0, 0, -0.5 + 0.5}};

        auto joint_mesh = linemesh(Vs, Es);
        label_surface(joint_mesh);  // visualize the joint

        // Use multi-instance API:
        vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot, left_geo_slot};
        vector<IndexT> l_instance_id = {0, 1};
        vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot, right_geo_slot};
        vector<IndexT> r_instance_id   = {0, 1};
        vector<Float>  strength_ratios = {100.0, 100.0};

        abrj.apply_to(joint_mesh, l_geo_slots, l_instance_id, r_geo_slots, r_instance_id, strength_ratios);

        auto joints = scene.objects().create("joint");
        joints->geometries().create(joint_mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(
        fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
