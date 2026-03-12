#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_revolute_joint_external_force.h>

TEST_CASE("74_abd_revolute_joint_external_force", "[abd][joint][external_force]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs     = std::filesystem;
    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                            = test::Scene::default_config();
    config["gravity"]                      = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"]            = true;
    config["line_search"]["report_energy"] = true;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    scene.contact_tabular().default_model(0.5, 1.0_GPa);

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.4);
    SimplicialComplexIO io{pre_transform};

    // Create objects
    auto left_link  = scene.objects().create("left");
    auto right_link = scene.objects().create("right");

    AffineBodyConstitution abd;
    SimplicialComplex      abd_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(abd_mesh, 100.0_MPa);
    label_surface(abd_mesh);

    // Left body: fixed
    SimplicialComplex left_mesh = abd_mesh;
    {
        left_mesh.instances().resize(1);
        Transform t0 = Transform::Identity();
        t0.translate(Vector3::UnitX() * -0.6);
        view(left_mesh.transforms())[0] = t0.matrix();

        auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;
    }
    auto [left_geo_slot, left_rest_geo_slot] = left_link->geometries().create(left_mesh);

    // Right body: free
    SimplicialComplex right_mesh = abd_mesh;
    {
        right_mesh.instances().resize(1);
        Transform t0 = Transform::Identity();
        t0.translate(Vector3::UnitX() * 0.6);
        view(right_mesh.transforms())[0] = t0.matrix();

        auto is_fixed = right_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 0;
    }
    auto [right_geo_slot, right_rest_geo_slot] = right_link->geometries().create(right_mesh);

    // Create revolute joint along Z axis (single edge)
    vector<Vector2i> Es = {{0, 1}};
    vector<Vector3>  Vs = {Vector3{0.0, 0.0, -0.5}, Vector3{0.0, 0.0, 0.5}};

    auto joint_mesh = linemesh(Vs, Es);
    label_surface(joint_mesh);

    // Apply revolute joint (single-instance API)
    AffineBodyRevoluteJoint          revolute_joint;
    vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot};
    vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot};
    revolute_joint.apply_to(joint_mesh, span{l_geo_slots}, span{r_geo_slots}, 100.0);

    // Apply external torque around the joint axis (initially zero)
    AffineBodyRevoluteJointExternalBodyForce ext_torque;
    ext_torque.apply_to(joint_mesh, Float{0});

    auto revolute_joint_object = scene.objects().create("revolute_joint");
    revolute_joint_object->geometries().create(joint_mesh);

    // Animator: dynamically update external torque
    scene.animator().insert(
        *revolute_joint_object,
        [](Animation::UpdateInfo& info)
        {
            for(auto& geo_slot : info.geo_slots())
            {
                if(!geo_slot)
                    continue;

                auto& geo = geo_slot->geometry();
                auto* sc  = geo.as<SimplicialComplex>();
                if(!sc)
                    continue;

                auto external_torque = sc->edges().find<Float>("external_torque");
                if(!external_torque)
                    continue;

                auto is_constrained =
                    sc->edges().find<IndexT>("external_torque/is_constrained");
                if(is_constrained)
                {
                    auto constrained_view = view(*is_constrained);
                    std::fill(constrained_view.begin(), constrained_view.end(), 1);
                }

                // Apply torque for first 50 frames, then reverse
                Float torque_value = (info.frame() <= 50) ? -1000.0 : 1000.0;

                std::ranges::fill(view(*external_torque), torque_value);

                spdlog::info("Frame {} external_torque: {:.2f}", info.frame(), torque_value);
            }
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
