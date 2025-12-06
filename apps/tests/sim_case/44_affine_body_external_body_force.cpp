#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_external_force.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <filesystem>
#include <fstream>

TEST_CASE("44_affine_body_external_body_force", "[abd][external_force]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    // Disable built-in gravity since we're using external force
    config["gravity"]           = Vector3{0, 0, 0};
    config["contact"]["enable"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // Create constitutions
        AffineBodyConstitution      abd;
        AffineBodyExternalBodyForce ext_force;

        // Setup contact
        scene.contact_tabular().default_model(0.5, 1e9);
        auto default_element = scene.contact_tabular().default_element();

        // Load cube mesh
        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0)     = 0.2;
        pre_trans(1, 1)     = 0.2;
        pre_trans(2, 2)     = 0.2;

        SimplicialComplexIO io{pre_trans};
        auto cube = io.read(fmt::format("{}/cube.msh", tetmesh_dir));

        // Process surface
        label_surface(cube);
        label_triangle_orient(cube);
        auto cube_processed = flip_inward_triangles(cube);

        // Create object with single cube
        auto object = scene.objects().create("cube");

        cube_processed.instances().resize(1);

        // Apply constitutions
        abd.apply_to(cube_processed, 1e8);  // stiffness

        // Apply external force - initially zero, will be set by animator
        Vector12 initial_force = Vector12::Zero();
        ext_force.apply_to(cube_processed, initial_force);

        default_element.apply_to(cube_processed);

        // Set transform - position at y=0.5
        auto      trans_view = view(cube_processed.transforms());
        Transform t          = Transform::Identity();
        t.translation()      = Vector3(0, 0.5, 0);
        trans_view[0]        = t.matrix();

        object->geometries().create(cube_processed);

        // Add animator for combined orbital and spinning motion
        scene.animator().insert(
            *object,
            [](Animation::UpdateInfo& info)
            {
                Float time = info.dt() * info.frame();

                // Rotation parameters
                Float orbit_speed     = 0.2f;   // rad/s
                Float spin_speed      = 0.1f;   // rad/s
                Float force_magnitude = 10.0f;  // N

                // Calculate orbital force direction (in XZ plane)
                Float orbit_angle = orbit_speed * time;
                Vector3 force_direction(std::cos(orbit_angle), 0.0f, std::sin(orbit_angle));
                Vector3 force_3d = force_direction * force_magnitude;

                // Calculate affine force for spinning (rotation around Y axis)
                Float    omega_y = spin_speed * 0.1f;
                Vector12 force;
                force.segment<3>(0) = force_3d;  // Linear force
                force.segment<9>(3).setZero();
                force(5) = omega_y;   // f_a13
                force(9) = -omega_y;  // f_a31

                // Update force for all geometries
                for(auto& geo_slot : info.geo_slots())
                {
                    auto& geo = geo_slot->geometry();
                    auto* sc  = geo.as<SimplicialComplex>();
                    if(!sc)
                        continue;

                    auto force_attr = sc->instances().find<Vector12>("external_force");
                    if(!force_attr)
                        continue;

                    // Set is_constrained to enable external force
                    auto is_constrained =
                        sc->instances().find<IndexT>(builtin::is_constrained);
                    if(is_constrained)
                    {
                        auto is_constrained_view = view(*is_constrained);
                        is_constrained_view[0]   = 1;
                    }

                    auto force_view = view(*force_attr);
                    for(auto& f : force_view)
                        f = force;
                }
            });
    }

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 100)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}
