#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <filesystem>
#include <fstream>

TEST_CASE("33_discrete_shell_bending", "[fem]")
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
    // config["line_search"]["report_energy"]  = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        NeoHookeanShell      nhs;
        DiscreteShellBending dsb;
        scene.constitution_tabular().insert(nhs);
        auto& default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("shell");

        vector<Vector3> Vs = {
            Vector3{1, 0, -1},   //
            Vector3{-1, 0, -1},  //
            Vector3{-1, 0, 1},   //
            Vector3{1, 0, 1}     //
        };

        vector<Vector3i> Fs = {
            Vector3i{0, 1, 2},  //
            Vector3i{0, 2, 3}   //
        };

        std::transform(Vs.begin(),
                       Vs.end(),
                       Vs.begin(),
                       [&](const Vector3& v)
                       { return v * 0.3 + Vector3::UnitY() * 0.35; });

        auto mesh = trimesh(Vs, Fs);

        label_surface(mesh);

        auto parm = ElasticModuli::youngs_poisson(10.0_MPa, 0.49);
        nhs.apply_to(mesh, parm);
        dsb.apply_to(mesh, 5.0_kPa);
        default_contact.apply_to(mesh);

        auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);
        is_fixed_view[0]   = 1;
        is_fixed_view[1]   = 1;
        is_fixed_view[2]   = 1;

        object->geometries().create(mesh);
    }

    world.init(scene); REQUIRE(world.is_valid());
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