#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body.h>
#include <fstream>


TEST_CASE("dump_recover", "[world]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config       = Scene::default_config();
    config["gravity"] = Vector3{0, -9.8, 0};

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
        scene.constitution_tabular().insert(abd);
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto& default_contact = scene.contact_tabular().default_element();

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto object = scene.objects().create("cubes");

        auto cube_mesh = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

        label_surface(cube_mesh);
        label_triangle_orient(cube_mesh);

        constexpr SizeT N = 4;
        cube_mesh.instances().resize(N);
        abd.apply_to(cube_mesh, 100.0_MPa);
        default_contact.apply_to(cube_mesh);

        auto trans_view = view(cube_mesh.transforms());
        auto is_fixed   = cube_mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);


        for(SizeT i = 0; i < N; i++)
        {
            Transform t      = Transform::Identity();
            t.translation()  = Vector3::UnitY() * 0.35 * i;
            trans_view[i]    = t.matrix();
            is_fixed_view[i] = 0;
        }

        is_fixed_view[0] = 1;

        object->geometries().create(cube_mesh);
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    SizeT frame = 1;

    if(world.recover())
    {
        frame = world.frame() + 1;  // calculate from the next frame
    }

    for(int i = 0; i < 10; i++)  // always run 10 frames from the recovered frame
    {
        world.advance();
        world.retrieve();
        world.dump();

        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}