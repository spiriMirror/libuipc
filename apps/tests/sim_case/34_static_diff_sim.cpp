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

    spdlog::set_level(spdlog::level::err);

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["dt"]                            = 1.0;
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = false;
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


    // create constitution and contact model
    HookeanSpring hs;

    auto& default_contact = scene.contact_tabular().default_element();

    // create object
    auto object = scene.objects().create("rods");

    constexpr int   n = 2;
    vector<Vector3> Vs(n);
    for(int i = 0; i < n; i++)
    {
        Vs[i] = Vector3{0, 0, 0} - Vector3::UnitY() * 0.1 * i;
    }

    vector<Vector2i> Es(n - 1);
    for(int i = 0; i < n - 1; i++)
    {
        Es[i] = Vector2i{i, i + 1};
    }

    auto mesh = linemesh(Vs, Es);
    label_surface(mesh);
    hs.apply_to(mesh, 40.0_MPa);

    // connect every kappa to the diff_sim.parameters()[0]
    mesh.edges().create<IndexT>("diff/kappa", 0);

    default_contact.apply_to(mesh);

    auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
    auto is_fixed_view = view(*is_fixed);
    is_fixed_view[0]   = 1;

    auto [geo_slot, rest_geo_slot] = object->geometries().create(mesh);


    world.init(scene);

    SceneIO sio{scene};
    world.dump();  // dump the initial state

    constexpr SizeT Frame = 1;

    Float aim_y = -0.4;  // we want the vertex 1 to reach this y position in 3 frames
    Float error = 0.001;


    auto YofX = [](const Eigen::VectorXd& X) -> Float { return X.tail<3>().y(); };

    // loss function: L = losK * (y - aim_y)^2

    auto losK = 1e5;

    auto Loss = [&](const Eigen::VectorXd& X, Float aim_y) -> Float
    {
        auto y = YofX(X);
        return 0.5 * losK * (y - aim_y) * (y - aim_y);
    };

    auto dLossdY = [&](Float y, Float aim_y) -> Float
    { return losK * (y - aim_y); };

    auto dLossdX = [&](const Eigen::VectorXd& X, Float aim_y) -> Eigen::VectorXd
    {
        Eigen::VectorXd ret = Eigen::VectorXd::Zero(X.size());
        auto            y   = YofX(X);
        ret.tail<3>().y()   = dLossdY(y, aim_y);
        return ret;
    };

    // fill guess value
    auto diff_parm_view = view(diff_sim.parameters());
    std::ranges::fill(diff_parm_view, 100.0_Pa);

    vector<Float> Xbuffer;
    Xbuffer.reserve(Frame * 3 * mesh.vertices().size());

    for(auto epoch : range(100))
    {
        world.recover(0);  // recover the initial state
        world.retrieve();  // retrieve the initial state
        sio.write_surface(fmt::format(
            "{}scene_surface.{}.{}.obj", this_output_path, epoch, world.frame()));

        diff_sim.clear();  // clear the diff_sim gradient
        // broadcast the updated parameters to geometry attributes
        diff_sim.parameters().broadcast();


        Xbuffer.clear();
        while(world.frame() < Frame)
        {
            world.advance();
            world.backward();
            world.retrieve();
            sio.write_surface(fmt::format(
                "{}scene_surface.{}.{}.obj", this_output_path, epoch, world.frame()));

            auto pos_view = geo_slot->geometry().positions().view();
            for(auto p : pos_view)
            {
                Xbuffer.push_back(p.x());
                Xbuffer.push_back(p.y());
                Xbuffer.push_back(p.z());
            }
        }

        Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd>(Xbuffer.data(), Xbuffer.size());

        // record the current parameters
        auto          parm_view = view(diff_sim.parameters());
        vector<Float> current_parm(parm_view.size());
        std::ranges::copy(parm_view, current_parm.begin());

        auto loss = Loss(X, aim_y);
        std::cout << "epoch: " << epoch << ", loss: " << loss << std::endl;

        auto pGpP       = scene.diff_sim().pGpP();
        auto dense_pGpP = pGpP.to_dense();

        auto H       = scene.diff_sim().H();
        auto dense_H = H.to_dense();

        Eigen::VectorXd dXdP = dense_H.ldlt().solve(-dense_pGpP);

        std::cout << "dXdP:\n" << dXdP << std::endl;

        Eigen::VectorXd dLdX = dLossdX(X, aim_y);

        std::cout << "dLdX:\n" << dLdX << std::endl;

        Eigen::VectorXd dLdP = dLdX.transpose() * dXdP;

        std::cout << "dLdP:\n" << dLdP << std::endl;

        Float Y = YofX(X);
        std::cout << "Y:" << Y << std::endl;

        sio.write_surface(fmt::format("{}scene_surface.epoch.{}.obj", this_output_path, epoch));

        if(std::abs(Y - aim_y) < error)
        {
            std::cout << "converged, epoch:" << epoch << std::endl;
            break;
        }

        Float alpha = 1.0;

        while(true)
        {
            // gradient descent
            for(int i = 0; i < parm_view.size(); i++)
            {
                parm_view[i] -= alpha * dLdP(i);
            }
            diff_sim.parameters().broadcast();
            std::cout << "kappa:" << parm_view[0] << std::endl;

            Xbuffer.clear();
            world.recover(0);

            while(world.frame() < Frame)
            {
                world.advance();
                world.retrieve();

                auto pos_view = geo_slot->geometry().positions().view();
                for(auto p : pos_view)
                {
                    Xbuffer.push_back(p.x());
                    Xbuffer.push_back(p.y());
                    Xbuffer.push_back(p.z());
                }
            }

            Eigen::VectorXd X =
                Eigen::Map<Eigen::VectorXd>(Xbuffer.data(), Xbuffer.size());

            auto new_loss = Loss(X, aim_y);

            std::cout << "epoch: " << epoch << ", loss: " << new_loss << "/"
                      << loss << ", alpha: " << alpha << std::endl;

            if(new_loss <= loss)
            {
                loss = new_loss;
                break;
            }
            else
            {
                alpha *= 0.5;
            }
        };
    }
}