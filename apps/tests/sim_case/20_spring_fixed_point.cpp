#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/hookean_spring.h>
#include <filesystem>
#include <fstream>

TEST_CASE("20_spring_fixed_point", "[fem]")
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
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["report_energy"]  = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        HookeanSpring hs;
        scene.constitution_tabular().insert(hs);
        auto& default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("shell");


        constexpr int   n = 15;
        vector<Vector3> Vs(n);
        for(int i = 0; i < n; i++)
        {
            Vs[i] = Vector3{0, 0, 0} + Vector3::UnitZ() * i;
        }

        vector<Vector2i> Es(n - 1);
        for(int i = 0; i < n - 1; i++)
        {
            Es[i] = Vector2i{i, i + 1};
        }

        std::transform(Vs.begin(),
                       Vs.end(),
                       Vs.begin(),
                       [&](const Vector3& v)
                       { return v * 0.03 + Vector3::UnitY() * 0.1; });

        auto mesh = linemesh(Vs, Es);

        label_surface(mesh);

        hs.apply_to(mesh, 40.0_MPa);
        default_contact.apply_to(mesh);

        auto is_fixed        = mesh.vertices().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view   = view(*is_fixed);
        is_fixed_view[0]     = 1;
        is_fixed_view[n - 1] = 1;

        object->geometries().create(mesh);

        //auto g = ground(0.0);
        //object->geometries().create(g);
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 100; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}