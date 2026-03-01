#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_shell.h>

TEST_CASE("63_abd_codim_shell_gravity", "[abd][codim]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        AffineBodyShell abd_shell;

        auto object = scene.objects().create("shell");

        vector<Vector3> Vs = {
            Vector3{-0.5, 0, -0.5},
            Vector3{ 0.5, 0, -0.5},
            Vector3{ 0.5, 0,  0.5},
            Vector3{-0.5, 0,  0.5}};

        vector<Vector3i> Fs = {
            Vector3i{0, 1, 2},
            Vector3i{0, 2, 3}};

        auto mesh = trimesh(Vs, Fs);
        label_surface(mesh);

        mesh.instances().resize(2);

        abd_shell.apply_to(mesh, 100.0_MPa, 1e3, 0.01);

        auto trans_view    = view(mesh.transforms());
        auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);

        {
            Transform t     = Transform::Identity();
            t.translation() = Vector3{0, 1.0, 0};
            trans_view[0]   = t.matrix();
            is_fixed_view[0] = 0;
        }

        {
            Transform t     = Transform::Identity();
            t.translation() = Vector3{1.5, 1.0, 0};
            trans_view[1]   = t.matrix();
            is_fixed_view[1] = 1;
        }

        object->geometries().create(mesh);
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



