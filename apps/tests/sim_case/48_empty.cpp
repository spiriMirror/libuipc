#include <app/catch2.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/empty.h>
#include <filesystem>
#include <fstream>

TEST_CASE("48_empty", "[fem]")
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

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = false;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["newton"]["velocity_tol"]        = 0.05;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        Empty empty;
        auto  default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("empty");


        constexpr int   n = 1;
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

        empty.apply_to(mesh);
        default_contact.apply_to(mesh);
        object->geometries().create(mesh);

        auto g = ground(0.0);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 100; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}