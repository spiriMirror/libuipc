#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/builtin/constants.h>

TEST_CASE("49_abd_adaptive_kappa", "[abd]")
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
        AffineBodyConstitution abd;
        scene.contact_tabular().default_model(0, builtin::adaptive);
        auto default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        {
            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](auto& v) { return v * 0.3; });

            auto mesh1 = tetmesh(Vs, Ts);

            label_surface(mesh1);
            label_triangle_orient(mesh1);

            mesh1.instances().resize(1);
            abd.apply_to(mesh1, 100.0_MPa);
            default_contact.apply_to(mesh1);

            auto trans_view = view(mesh1.transforms());
            auto is_fixed   = mesh1.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            {
                Transform t      = Transform::Identity();
                t.translation()  = Vector3::UnitY() * 0.305;
                trans_view[0]    = t.matrix();
                is_fixed_view[0] = 0;
            }

            object->geometries().create(mesh1);
        }

        {

            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](auto& v) { return v * 0.3; });


            auto mesh2 = tetmesh(Vs, Ts);

            label_surface(mesh2);
            label_triangle_orient(mesh2);

            mesh2.instances().resize(1);
            // apply constitution and contact model to the geometry
            abd.apply_to(mesh2, 100.0_MPa);
            default_contact.apply_to(mesh2);

            auto trans_view = view(mesh2.transforms());
            auto is_fixed   = mesh2.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            {
                Transform t      = Transform::Identity();
                trans_view[0]    = t.matrix();
                is_fixed_view[0] = 1;
            }

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
