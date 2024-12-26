#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <uipc/common/enumerate.h>

TEST_CASE("26_abd_ramp_ground", "[abd]")
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
    config["contact"]["friction"]["enable"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model

        AffineBodyConstitution abd;
        scene.constitution_tabular().insert(abd);


        auto& contact_tabular = scene.contact_tabular();
        contact_tabular.default_model(0.5, 1.0_GPa);
        auto& default_element = contact_tabular.default_element();

        constexpr SizeT N = 8;

        auto friction_rate_step = 1.0 / (N - 1);

        vector<ContactElement*> contact_elements(N);

        for(auto&& [i, e] : enumerate(contact_elements))
        {
            e = &contact_tabular.create(fmt::format("element{}", i));
            contact_tabular.insert(*e, default_element, friction_rate_step * i, 1.0_GPa);
        }


        // create object
        auto cubes = scene.objects().create("cube");
        {
            Transform pre_transform = Transform::Identity();
            pre_transform.scale(0.3);
            SimplicialComplexIO io{pre_transform};

            auto cube = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

            label_surface(cube);
            label_triangle_orient(cube);

            abd.apply_to(cube, 100.0_MPa);


            Float step    = 0.5;
            Float start_x = -step * (N - 1) / 2.0;

            for(SizeT i = 0; i < N; i++)
            {
                SimplicialComplex this_cube = cube;

                contact_elements[i]->apply_to(this_cube);

                auto      trans_view = view(this_cube.transforms());
                Transform t          = Transform::Identity();
                t.translate(Vector3{start_x + step * i, 1, -0.7});
                t.rotate(Eigen::AngleAxisd(30.0 * std::numbers::pi / 180.0,
                                           Vector3::UnitX()));

                trans_view[0] = t.matrix();

                cubes->geometries().create(this_cube);
            }
        }

        auto object_ramp = scene.objects().create("ramp");
        {
            // rotate 30 degree
            Float   theta = 30.0 * std::numbers::pi / 180.0;
            Vector3 N     = Vector3{0, std::cos(theta), std::sin(theta)};
            Vector3 P     = Vector3{0, 0, 0};
            auto    ramp  = halfplane(P, N);
            object_ramp->geometries().create(ramp);
        }
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