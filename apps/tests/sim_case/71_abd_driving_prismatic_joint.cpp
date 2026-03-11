#include "uipc/constitution/affine_body_driving_prismatic_joint.h"
#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>

TEST_CASE("71_abd_driving_prismatic_joint", "[abd joint]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
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

    AffineBodyDrivingPrismaticJoint driving_prismatic_joint;
    driving_prismatic_joint.apply_to(joint_mesh, span{strength_ratios});

    auto prismatic_joint_object = scene.objects().create("prismatic_joint");
    prismatic_joint_object->geometries().create(joint_mesh);

    scene.animator().insert(
        *prismatic_joint_object,
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

                // Find the "distance" attribute on edges (in radians), skip if not exists
                auto distances = sc->edges().find<Float>("distance");
                if(!distances)
                    continue;
                auto distances_view = view(*distances);

                // Log angle values in degrees for current frame (easier to read)
                for(size_t i = 0; i < distances_view.size(); ++i)
                {
                    spdlog::info("Frame {} Edge {} distance: {:.2f} ",
                                 info.frame(),
                                 i,
                                 distances_view[i]);
                }

                // 1. Enable is_constrained
                auto is_constrained = sc->edges().find<IndexT>(builtin::is_constrained);
                if(is_constrained)
                {
                    auto constrained_view = view(*is_constrained);
                    std::fill(constrained_view.begin(), constrained_view.end(), 1);
                }
                // Frame-based logic: different behavior for first 50 frames vs later frames
                if(info.frame() <= 50)
                {
                    // 2. Update aim_distance decrease by 10m/s
                    auto aim_distances = sc->edges().find<Float>("aim_distance");
                    if(aim_distances)
                    {
                        auto aim_distance_view = view(*aim_distances);
                        // Ensure array size consistency to prevent out-of-bounds access
                        const size_t valid_size =
                            std::min(distances_view.size(), aim_distance_view.size());
                        for(size_t i = 0; i < valid_size; ++i)
                        {
                            aim_distance_view[i] = distances_view[i] - info.dt() * 10;
                        }
                    }
                }
                else
                {
                    // 3. Update aim_distance increase by 10 m/s
                    auto aim_distances = sc->edges().find<Float>("aim_distance");
                    if(aim_distances)
                    {
                        auto aim_distance_view = view(*aim_distances);
                        // Ensure array size consistency to prevent out-of-bounds access
                        const size_t valid_size =
                            std::min(distances_view.size(), aim_distance_view.size());
                        for(size_t i = 0; i < valid_size; ++i)
                        {
                            aim_distance_view[i] = distances_view[i] + info.dt() * 10;
                        }
                    }
                }
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
