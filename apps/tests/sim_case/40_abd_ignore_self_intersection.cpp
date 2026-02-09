#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("40_abd_ignore_self_intersection", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    std::string trimesh_dir{AssetDir::trimesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config       = test::Scene::default_config();
    config["gravity"] = Vector3{0, -9.8, 0};
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
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
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
