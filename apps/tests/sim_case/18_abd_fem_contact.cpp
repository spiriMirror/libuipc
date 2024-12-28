#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/arap.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("18_abd_fem_contact", "[abd_fem]")
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

    auto config = Scene::default_config();

    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["line_search"]["report_energy"]  = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean       snh;
        AffineBodyConstitution abd;

        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto& default_element = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("cubes");

        auto lower_mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

        label_surface(lower_mesh);
        label_triangle_orient(lower_mesh);

        SimplicialComplex upper_mesh = lower_mesh;

        auto parm = ElasticModuli::youngs_poisson(20.0_kPa, 0.49);
        snh.apply_to(lower_mesh, parm);
        abd.apply_to(upper_mesh, 1.0_MPa);
        // abd.apply_to(lower_mesh, 1.0_MPa);

        constexpr SizeT N = 6;

        for(SizeT i = 0; i < N; i++)
        {
            if(i % 2 == 0)
            {
                SimplicialComplex l = lower_mesh;
                {
                    auto pos_v = view(l.positions());
                    for(auto& p : pos_v)
                        p.y() += 1.2 * i;
                }
                object->geometries().create(l);
            }
            else
            {
                SimplicialComplex u = upper_mesh;
                {
                    auto pos_v = view(u.positions());
                    for(auto& p : pos_v)
                        p.y() += 1.2 * i;
                }
                object->geometries().create(u);
            }
        }

        auto g = ground(-1.2);
        object->geometries().create(g);
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