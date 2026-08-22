#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("0_abd_gravity", "[abd]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};

    std::string this_output_path;
    std::string contact_constitution;

    SECTION("ipc")
    {
        this_output_path =
            fmt::format("{}ipc/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));
        contact_constitution = "ipc";
    };

    SECTION("al-ipc")
    {
        this_output_path =
            fmt::format("{}al-ipc/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));
        contact_constitution = "al-ipc";
    };

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    config["gravity"]                 = Vector3{0, -9.8, 0};
    config["contact"]["enable"]       = false;  // disable contact
    config["contact"]["constitution"] = contact_constitution;

    test::Scene::dump_config(config, this_output_path);


    Scene                    scene{config};
    S<SimplicialComplexSlot> geo_slot1;
    S<SimplicialComplexSlot> geo_slot2;
    {
        AffineBodyConstitution abd;

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        {
            vector<Vector3> Vs = {Vector3{0, 0, 1},
                                  Vector3{0, -1, 0},
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

            auto trans_view = view(mesh1.transforms());
            auto is_fixed   = mesh1.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            {
                Transform t      = Transform::Identity();
                t.translation()  = Vector3::UnitY() * 1;
                trans_view[0]    = t.matrix();
                is_fixed_view[0] = 0;
            }

            auto [slot1, rest_slot1] = object->geometries().create(mesh1);
            geo_slot1                = slot1;
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

            auto trans_view = view(mesh2.transforms());
            auto is_fixed   = mesh2.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            {
                Transform t      = Transform::Identity();
                trans_view[0]    = t.matrix();
                is_fixed_view[0] = 0;
            }

            auto [slot2, rest_slot2] = object->geometries().create(mesh2);
            geo_slot2                = slot2;
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }

    // Numerical regression: pure-gravity free fall under BDF1 has a closed
    // form. Starting from rest, after n frames the fall distance is
    // g * dt^2 * n*(n+1)/2 (verified against the exported surface geometry).
    {
        constexpr Float g    = 9.8;
        constexpr Float dt   = 0.01;
        constexpr Float n    = 50;
        const Float     fall = g * dt * dt * (n * (n + 1) / 2);  // = 1.2495

        // ABD geometries store the *local* (rest) vertex positions; the
        // simulated rigid state lives in the per-instance `transforms()`.
        // The y-translation of the instance transform is the centroid motion.
        auto translation_y = [](S<SimplicialComplexSlot>& slot)
        {
            auto* sc = slot->geometry().as<SimplicialComplex>();
            REQUIRE(sc);
            auto trans = sc->transforms().view();
            REQUIRE(trans.size() == 1);
            const Matrix4x4& T = trans[0];
            for(int i = 0; i < 4; ++i)
                for(int j = 0; j < 4; ++j)
                    REQUIRE(std::isfinite(T(i, j)));
            return T(1, 3);
        };

        // mesh1 initial centroid y-offset = 1.0 (instance transform translation)
        REQUIRE(std::abs(translation_y(geo_slot1) - (1.0 - fall)) < 0.05);
        // mesh2 initial centroid y-offset = 0.0
        REQUIRE(std::abs(translation_y(geo_slot2) - (0.0 - fall)) < 0.05);
    }
}
