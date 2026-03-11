#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_driving_revolute_joint.h>

TEST_CASE("71_abd_driving_revolute_joint", "[abd joint]")
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
            view(*is_fixed)[1] = 1;  // instance 1 not fixed
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
            view(*is_fixed)[1] = 0;  // fix instance 1
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

        // driving
        AffineBodyDrivingRevoluteJoint abd_driving_revolute_joint;
        abd_driving_revolute_joint.apply_to(joint_mesh, span{strength_ratios});

        auto revolute_joints = scene.objects().create("revolute_joint");
        revolute_joints->geometries().create(joint_mesh);

        auto to_degrees = [](auto radians)
        { return radians * (180.0 / std::numbers::pi); };
        scene.animator().insert(
            *revolute_joints,
            [=](Animation::UpdateInfo& info)
            {
                // Iterate through all geometry slots in the animation update context
                for(auto& geo_slot : info.geo_slots())
                {
                    // Null pointer protection for geometry slot
                    if(!geo_slot)
                        continue;

                    auto& geo = geo_slot->geometry();
                    auto* sc  = geo.as<SimplicialComplex>();
                    // Skip if geometry is not a SimplicialComplex
                    if(!sc)
                        continue;

                    // Find the "angle" attribute on edges (in radians), skip if not exists
                    auto angles = sc->edges().find<Float>("angle");
                    if(!angles)
                        continue;
                    auto angles_view = view(*angles);

                    // Log angle values in degrees for current frame (easier to read)
                    for(size_t i = 0; i < angles_view.size(); ++i)
                    {
                        spdlog::info("Frame {} Edge {} angle: {:.2f} degrees ",
                                     info.frame(),
                                     i,
                                     to_degrees(angles_view[i]));
                    }

                    // 1. Enable is_constrained (set all values to 1)
                    auto is_constrained = sc->edges().find<IndexT>(builtin::is_constrained);
                    if(is_constrained)  // Check if attribute exists to avoid crash
                    {
                        auto constrained_view = view(*is_constrained);
                        std::fill(constrained_view.begin(), constrained_view.end(), 1);
                    }
                    // Frame-based logic: different behavior for first 20 frames vs later frames
                    if(info.frame() <= 50)
                    {
                        // 2. Update aim_angle: decrease by 4 rad/s (relative to current angle)
                        auto aim_angle = sc->edges().find<Float>("aim_angle");
                        if(aim_angle)
                        {
                            auto aim_angle_view = view(*aim_angle);
                            // Ensure array size consistency to prevent out-of-bounds access
                            const size_t valid_size =
                                std::min(angles_view.size(), aim_angle_view.size());
                            for(size_t i = 0; i < valid_size; ++i)
                            {
                                aim_angle_view[i] = angles_view[i] - info.dt() * 5;
                            }
                        }
                    }
                    else
                    {
                        // 3. Update aim_angle: increase by 4 rad/s (relative to current angle)
                        auto aim_angle = sc->edges().find<Float>("aim_angle");
                        if(aim_angle)
                        {
                            auto aim_angle_view = view(*aim_angle);
                            // Ensure array size consistency to prevent out-of-bounds access
                            const size_t valid_size =
                                std::min(angles_view.size(), aim_angle_view.size());
                            for(size_t i = 0; i < valid_size; ++i)
                            {
                                aim_angle_view[i] = angles_view[i] + info.dt() * 10;
                            }
                        }
                    }
                }
            });
    }

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
