#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/constitution/kirchhoff_rod_bending.h>
#include <filesystem>
#include <fstream>

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
        mesh.edges().create<IndexT>("diff_parm/kappa", 0);

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
        world.retrieve();

        // get partial Gradient partial Parameters
        auto pGpP = scene.diff_sim().pGpP();
        // get system hessian
        auto H = scene.diff_sim().H();

        // try to optimize the parameters
        Eigen::MatrixXd pGpP_mat;  // == pGpP.to_dense()
        Eigen::MatrixXd H_mat;     // == H.to_dense()

        // solve the linear system to get dXdP
        // Eigen::MatrixXd dXdP = H_mat.colPivHouseholderQr().solve(pGpP_mat);

        // ... gradient descent ...

        auto diff_parm_view = view(diff_sim.parameters());

        Eigen::VectorXd dP;
        dP.resize(diff_parm_view.size());
        dP.setZero();

        // update the parameters
        std::transform(diff_parm_view.begin(),
                       diff_parm_view.end(),
                       dP.begin(),
                       diff_parm_view.begin(),
                       [](const Float& p, const Float& dp) { return p - dp; });

        // broadcast the updated parameters to geometry attributes
        diff_sim.parameters().broadcast();

        sio.write_surface(fmt::format("{}scene_surface_epoch{}.obj", this_output_path, epoch));
    }
}