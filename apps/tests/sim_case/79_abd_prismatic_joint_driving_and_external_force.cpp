#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/affine_body_driving_prismatic_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint_external_force.h>

TEST_CASE("79_abd_prismatic_joint_driving_and_external_force", "[abd][joint][driving][external_force]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    uipc::logger::set_level(uipc::Logger::Level::warn);
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

    // Prismatic joint along a diagonal axis
    vector<Vector2i> Es = {{0, 1}};
    vector<Vector3>  Vs = {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 1.0}};

    auto joint_mesh = linemesh(Vs, Es);
    label_surface(joint_mesh);

    // 1) Base prismatic joint
    AffineBodyPrismaticJoint         prismatic_joint;
    vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot};
    vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot};
    prismatic_joint.apply_to(joint_mesh, span{l_geo_slots}, span{r_geo_slots}, 100.0);

    // 2) Driving constitution (active during frames 1..100)
    AffineBodyDrivingPrismaticJoint driving;
    vector<Float>                   driving_strength = {100.0};
    driving.apply_to(joint_mesh, span{driving_strength});

    // 3) External force constitution (active during frames 101..200)
    AffineBodyPrismaticJointExternalForce ext_force;
    ext_force.apply_to(joint_mesh, Float{0});

    auto joint_object = scene.objects().create("prismatic_joint");
    joint_object->geometries().create(joint_mesh);

    constexpr IndexT kDrivingEndFrame = 100;

    scene.animator().insert(
        *joint_object,
        [=](Animation::UpdateInfo& info)
        {
            for(auto& geo_slot : info.geo_slots())
            {
                if(!geo_slot)
                    continue;

                auto& geo = geo_slot->geometry();
                auto* sc  = geo.as<SimplicialComplex>();
                if(!sc)
                    continue;

                bool driving_phase = (info.frame() <= kDrivingEndFrame);

                // --- driving attributes ---
                auto driving_is_constrained =
                    sc->edges().find<IndexT>("driving/is_constrained");
                auto aim_distance = sc->edges().find<Float>("aim_distance");
                auto distances    = sc->edges().find<Float>("distance");

                if(driving_is_constrained)
                {
                    auto v = view(*driving_is_constrained);
                    std::fill(v.begin(), v.end(), driving_phase ? 1 : 0);
                }

                if(driving_phase && aim_distance && distances)
                {
                    auto aim_view  = view(*aim_distance);
                    auto dist_view = view(*distances);
                    const Float velocity = (info.frame() <= 50) ? -10.0f : 10.0f;
                    for(size_t i = 0; i < std::min(aim_view.size(), dist_view.size()); ++i)
                        aim_view[i] = dist_view[i] + info.dt() * velocity;
                }

                // --- external force attributes ---
                auto ext_force_is_constrained =
                    sc->edges().find<IndexT>("external_force/is_constrained");
                auto external_force = sc->edges().find<Float>("external_force");

                if(ext_force_is_constrained)
                {
                    auto v = view(*ext_force_is_constrained);
                    std::fill(v.begin(), v.end(), driving_phase ? 0 : 1);
                }

                if(!driving_phase && external_force)
                {
                    Float force_value = (info.frame() <= 150) ? -1000 : 1000.0;
                    std::ranges::fill(view(*external_force), force_value);
                    spdlog::info("Frame {} external_force: {:.2f}", info.frame(), force_value);
                }

                // --- logging ---
                if(distances)
                {
                    auto dist_view = view(*distances);
                    for(size_t i = 0; i < dist_view.size(); ++i)
                        printf("Frame {%zu} distance: {%f}\n", info.frame(), dist_view[i]);
                }
            }
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
