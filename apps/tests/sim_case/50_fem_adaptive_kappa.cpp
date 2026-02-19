#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/builtin/constants.h>

TEST_CASE("50_fem_adaptive_kappa", "[fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["report_energy"]  = true;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean snk;
        scene.contact_tabular().default_model(0, builtin::adaptive);
        auto default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts     = {Vector4i{0, 1, 2, 3}};
        ElasticModuli    moduli = ElasticModuli::youngs_poisson(20.0_kPa, 0.49);
        {
            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.400);
            t.scale(0.3);

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](const Vector3& v) -> Vector3 { return t * v; });


            auto mesh1 = tetmesh(Vs, Ts);

            label_surface(mesh1);
            label_triangle_orient(mesh1);

            snk.apply_to(mesh1, moduli);
            default_contact.apply_to(mesh1);

            object->geometries().create(mesh1);
        }

        {

            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            Transform t = Transform::Identity();
            t.scale(0.3);

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](const Vector3& v) -> Vector3 { return t * v; });


            auto mesh2 = tetmesh(Vs, Ts);

            label_surface(mesh2);
            label_triangle_orient(mesh2);

            snk.apply_to(mesh2, moduli);
            default_contact.apply_to(mesh2);

            auto is_fixed = mesh2.vertices().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            std::ranges::fill(is_fixed_view, 1);

            object->geometries().create(mesh2);
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
