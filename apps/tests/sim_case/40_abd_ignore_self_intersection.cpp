#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("40_abd_ignore_self_intersection", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    std::string trimesh_dir{AssetDir::trimesh_path()};
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
        auto default_contact = scene.contact_tabular().default_element();

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto object = scene.objects().create("cubes");

        auto mesh_0 = io.read(fmt::format("{}{}", trimesh_dir, "cube.obj"));
        mesh_0.instances().resize(2);
        // move instance 1 with translation
        auto t = Transform::Identity();
        t.translate(Vector3{0.15, 0.1, 0.15});
        view(mesh_0.transforms())[0] = t.matrix();

        // create meshes from instances
        auto meshes = apply_transform(mesh_0);
        // merge meshes into one simplicial complex (now we have in-body intersection)
        auto cube_mesh = merge(meshes);

        label_surface(cube_mesh);

        constexpr SizeT N = 2;
        cube_mesh.instances().resize(N);
        abd.apply_to(cube_mesh, 100.0_MPa);
        default_contact.apply_to(cube_mesh);

        auto trans_view = view(cube_mesh.transforms());
        auto is_fixed   = cube_mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);


        for(SizeT i = 0; i < N; i++)
        {
            Transform t = Transform::Identity();
            t.translation() = Vector3::UnitY() * 0.45 * i - Vector3::UnitZ() * 0.18 * i;
            trans_view[i]    = t.matrix();
            is_fixed_view[i] = 0;
        }

        is_fixed_view[0] = 1;

        object->geometries().create(cube_mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 50; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}