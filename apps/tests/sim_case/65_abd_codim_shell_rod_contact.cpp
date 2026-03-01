#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_shell.h>
#include <uipc/constitution/affine_body_rod.h>

TEST_CASE("65_abd_codim_shell_rod_contact", "[abd][codim]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        AffineBodyShell abd_shell;
        AffineBodyRod   abd_rod;

        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_contact = scene.contact_tabular().default_element();

        {
            auto shell_obj = scene.objects().create("shell");

            vector<Vector3> Vs = {
                Vector3{-0.6, 0, -0.6},
                Vector3{ 0.6, 0, -0.6},
                Vector3{ 0.6, 0,  0.6},
                Vector3{-0.6, 0,  0.6}};

            vector<Vector3i> Fs = {
                Vector3i{0, 1, 2},
                Vector3i{0, 2, 3}};

            auto mesh = trimesh(Vs, Fs);
            label_surface(mesh);

            abd_shell.apply_to(mesh, 100.0_MPa, 1e3, 0.02);
            default_contact.apply_to(mesh);

            auto trans_view    = view(mesh.transforms());
            auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            Transform t     = Transform::Identity();
            t.translation() = Vector3{0, 0.5, 0};
            trans_view[0]   = t.matrix();
            is_fixed_view[0] = 1;

            shell_obj->geometries().create(mesh);
        }

        {
            auto rod_obj = scene.objects().create("rod");

            vector<Vector3> Vs = {
                Vector3{-0.4, 0,  0.05},
                Vector3{-0.1, 0,  0.05},
                Vector3{ 0.2, 0,  0.05},
                Vector3{ 0.2, 0, -0.25}};

            vector<Vector2i> Es = {
                Vector2i{0, 1},
                Vector2i{1, 2},
                Vector2i{2, 3}};

            auto mesh = linemesh(Vs, Es);
            label_surface(mesh);

            abd_rod.apply_to(mesh, 100.0_MPa, 1e3, 0.03);
            default_contact.apply_to(mesh);

            auto trans_view    = view(mesh.transforms());
            auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            Transform t     = Transform::Identity();
            t.translation() = Vector3{0, 1.5, 0};
            trans_view[0]   = t.matrix();
            is_fixed_view[0] = 0;

            rod_obj->geometries().create(mesh);
        }

        {
            auto ground_obj = scene.objects().create("ground");
            ground_obj->geometries().create(ground(0.0));
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}

