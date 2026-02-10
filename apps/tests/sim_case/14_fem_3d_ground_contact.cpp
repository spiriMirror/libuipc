#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/arap.h>

TEST_CASE("14_fem_3d_ground_contact", "[fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};

    std::string this_output_path;
    std::string contact_constitution;

    SECTION("ipc")
    {
        this_output_path =
            fmt::format("{}ipc/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));
        contact_constitution = "ipc";
    };

    SECTION("al-ipc")
    {
        this_output_path =
            fmt::format("{}al-ipc/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));
        contact_constitution = "al-ipc";
    };

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["contact"]["constitution"]       = contact_constitution;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["line_search"]["report_energy"]  = false;
    test::Scene::dump_config(config, this_output_path);

    SimplicialComplexIO io;

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean snh;
        ARAP             arap;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 0, 1},
                               Vector3{0, -1, 0},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        auto mesh = tetmesh(Vs, Ts);

        label_surface(mesh);
        label_triangle_orient(mesh);

        auto parm = ElasticModuli::youngs_poisson(10.0_kPa, 0.49);
        snh.apply_to(mesh, parm);

        object->geometries().create(mesh);

        auto g = ground(-1.2);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 200)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}
