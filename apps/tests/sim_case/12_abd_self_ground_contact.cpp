#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("12_abd_self_ground_contact", "[abd]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["friction"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
        scene.constitution_tabular().insert(abd);
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_contact = scene.contact_tabular().default_element();

        Transform pre_trans = Transform::Identity();
        pre_trans.scale(0.3);
        SimplicialComplexIO io{pre_trans};
        auto cube = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

        //vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        //vector<Vector3>  Vs = {Vector3{0, 0, 1},
        //                       Vector3{0, -1, 0},
        //                       Vector3{-std::sqrt(3) / 2, 0, -0.5},
        //                       Vector3{std::sqrt(3) / 2, 0, -0.5}};

        //std::transform(
        //    Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.3; });

        //auto cube = tetmesh(Vs, Ts);

        // create object
        auto object = scene.objects().create("cubes");
        {
            label_surface(cube);
            label_triangle_orient(cube);

            constexpr auto N = 20;

            cube.instances().resize(N);

            auto trans = view(cube.transforms());

            for(int i = 0; i < N; i++)
            {
                Transform t = Transform::Identity();
                t.translate(Vector3::UnitY() * 0.4 * (i + 1));
                trans[i] = t.matrix();
            }

            abd.apply_to(cube, 100.0_MPa);

            object->geometries().create(cube);
        }

        // create a ground geometry
        ImplicitGeometry half_plane = ground(0.0);

        constexpr bool UseMeshGround = false;


        auto ground = scene.objects().create("ground");
        {
            if constexpr(UseMeshGround)
            {
                Transform pre_transform = Transform::Identity();
                pre_transform.scale(Vector3{40, 0.2, 40});

                SimplicialComplexIO io{pre_transform};
                io = SimplicialComplexIO{pre_transform};
                auto ground = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

                label_surface(ground);
                label_triangle_orient(ground);

                Transform transform = Transform::Identity();
                transform.translate(Vector3{12, -1.1, 0});
                view(ground.transforms())[0] = transform.matrix();
                abd.apply_to(ground, 10.0_MPa);

                auto is_fixed = ground.instances().find<IndexT>(builtin::is_fixed);
                view(*is_fixed)[0] = 1;

                auto ground_obj = scene.objects().create("ground");
                ground_obj->geometries().create(ground);
            }
            else
            {
                ground->geometries().create(half_plane);
            }
        }
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 75)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}