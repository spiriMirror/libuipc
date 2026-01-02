#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/particle.h>

TEST_CASE("21_particle_ground", "[fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["newton"]["velocity_tol"]        = 0.05;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        // create constitution and contact model
        Particle pt;
        auto     default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("particles");


        constexpr int   n = 10;
        vector<Vector3> Vs(n);
        for(int i = 0; i < n; i++)
        {
            Vs[i] = Vector3::UnitZ() * i;
        }

        std::transform(Vs.begin(),
                       Vs.end(),
                       Vs.begin(),
                       [&](const Vector3& v)
                       { return v * 0.05 + Vector3::UnitY() * 0.2; });

        auto mesh = pointcloud(Vs);

        label_surface(mesh);

        pt.apply_to(mesh);
        default_contact.apply_to(mesh);
        object->geometries().create(mesh);

        auto g = ground(0.0);
        object->geometries().create(g);
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