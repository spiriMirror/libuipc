#include <app/app.h>
#include <algorithm>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_driving_prismatic_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint_external_force.h>

/// AffineBodyDrivingPrismaticJoint + AffineBodyPrismaticJointExternalBodyForce on one slider.
/// Frames [0, 100): only driving (`driving/is_constrained` = 1, motor via `aim_distance`).
/// Frames [100, 200): only external force (`external_force/is_constrained` = 1).
TEST_CASE("82_abd_driving_prismatic_joint_and_external_force", "[abd][joint][driving][external_force]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;

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

    auto left_link  = scene.objects().create("left");
    auto right_link = scene.objects().create("right");

    AffineBodyConstitution abd;
    SimplicialComplex      abd_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(abd_mesh, 100.0_MPa);
    label_surface(abd_mesh);

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

    vector<Vector2i> Es = {{0, 1}};
    vector<Vector3>  Vs = {Vector3{0.0, 0.0, -0.5}, Vector3{0.0, 0.0, 0.5}};
    auto             joint_mesh = linemesh(Vs, Es);
    label_surface(joint_mesh);

    AffineBodyPrismaticJoint         prismatic_joint;
    vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot};
    vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot};
    prismatic_joint.apply_to(joint_mesh, span{l_geo_slots}, span{r_geo_slots}, 100.0);

    AffineBodyDrivingPrismaticJoint driving_joint;
    driving_joint.apply_to(joint_mesh, 100.0f);

    AffineBodyPrismaticJointExternalBodyForce ext_force;
    ext_force.apply_to(joint_mesh, Float{0});

    auto joint_object = scene.objects().create("prismatic_joint");
    joint_object->geometries().create(joint_mesh);

    scene.animator().insert(
        *joint_object,
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

                auto distances = sc->edges().find<Float>("distance");
                if(!distances)
                    continue;
                auto distances_view = view(*distances);

                const bool driving_phase = info.frame() < 100;

                auto driving_is_constrained =
                    sc->edges().find<IndexT>("driving/is_constrained");
                if(driving_is_constrained)
                {
                    auto v = view(*driving_is_constrained);
                    std::fill(v.begin(), v.end(), driving_phase ? IndexT{1} : IndexT{0});
                }

                auto ext_is_constrained =
                    sc->edges().find<IndexT>("external_force/is_constrained");
                if(ext_is_constrained)
                {
                    auto v = view(*ext_is_constrained);
                    std::fill(v.begin(), v.end(), driving_phase ? IndexT{0} : IndexT{1});
                }

                Float aim0         = 0.0f;
                auto  aim_distance = sc->edges().find<Float>("aim_distance");
                if(aim_distance && driving_phase)
                {
                    auto        aim_view    = view(*aim_distance);
                    const Float motor_speed = 1.5f;
                    for(SizeT i = 0; i < aim_view.size() && i < distances_view.size(); ++i)
                        aim_view[i] = distances_view[i] + info.dt() * motor_speed;
                    aim0 = aim_view[0];
                }
                else if(aim_distance)
                {
                    aim0 = view(*aim_distance)[0];
                }

                Float ext0          = 0.0f;
                auto external_force = sc->edges().find<Float>("external_force");
                if(external_force)
                {
                    if(driving_phase)
                    {
                        std::ranges::fill(view(*external_force), Float{0});
                    }
                    else
                    {
                        const SizeT k = info.frame() - 100;
                        // ext0          = (k < 50) ? -1000.0f : 1000.0f;
                        ext0 = 1000.0f;
                        std::ranges::fill(view(*external_force), ext0);
                    }
                }

                spdlog::info("Frame {} phase={} aim_distance[0]={:.4f} distance[0]={:.4f} ext_force[0]={:.2f}",
                             info.frame(),
                             driving_phase ? "driving" : "external",
                             aim0,
                             distances_view[0],
                             ext0);
            }
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 200)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
