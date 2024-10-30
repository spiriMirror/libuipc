#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/constitution/kirchhoff_rod_bending.h>
#include <filesystem>
#include <fstream>
#include <iostream>

TEST_CASE("34_static_diff_sim", "[diff_sim]")
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
    config["diff_sim"]["enable"]            = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    auto& diff_sim = scene.diff_sim();
    diff_sim.parameters().resize(1);
    // fill default value
    auto diff_parm_view = view(diff_sim.parameters());
    std::ranges::fill(diff_parm_view, 40.0_MPa);

    {
        // create constitution and contact model
        HookeanSpring hs;

        auto& default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("rods");

        constexpr int   n = 2;
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

        // connect every kappa to the diff_sim.parameters()[0]
        mesh.edges().create<IndexT>("diff/kappa", 0);

        default_contact.apply_to(mesh);

        auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);
        is_fixed_view[0]   = 1;

        object->geometries().create(mesh);
    }

    world.init(scene);

    // after init, we can update the diff_sim.parameters()
    diff_sim.parameters().broadcast();

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    world.dump();  // dump the initial state

    for(auto epoch : range(1))
    {
        world.advance();
        world.backward();
        world.retrieve();

        auto pGpP       = scene.diff_sim().pGpP();
        auto dense_pGpP = pGpP.to_dense();

        auto H       = scene.diff_sim().H();
        auto dense_H = H.to_dense();

        std::cout << "pGpP:\n" << dense_pGpP << std::endl;
        std::cout << "H:\n" << dense_H << std::endl;

        // solve the linear system to get dXdP
        Eigen::MatrixXd dXdP = dense_H.ldlt().solve(dense_pGpP);

        std::cout << "dXdP:\n" << dXdP << std::endl;

        // broadcast the updated parameters to geometry attributes
        diff_sim.parameters().broadcast();

        sio.write_surface(fmt::format("{}scene_surface_epoch{}.obj", this_output_path, epoch));
    }
}