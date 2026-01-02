#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <numbers>

TEST_CASE("24_abd_animated_translation", "[animation]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    Float dt                    = 0.01;
    config["gravity"]           = Vector3{0, 0, 0};
    config["contact"]["enable"] = false;  // disable contact
    config["line_search"]["max_iter"]   = 8;
    config["linear_system"]["tol_rate"] = 1e-3;
    config["dt"]                        = dt;
    test::Scene::dump_config(config, output_path);

    SimplicialComplexIO io;

    Scene scene{config};

    // create constitution and contact model
    AffineBodyConstitution  abd;
    SoftTransformConstraint stc;

    // create object
    auto object = scene.objects().create("tets");

    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
    vector<Vector3>  Vs = {Vector3{0, 1, 0},
                           Vector3{0, 0, 1},
                           Vector3{-std::sqrt(3) / 2, 0, -0.5},
                           Vector3{std::sqrt(3) / 2, 0, -0.5}};

    auto mesh = tetmesh(Vs, Ts);

    geometry::label_surface(mesh);
    geometry::label_triangle_orient(mesh);

    abd.apply_to(mesh, 1e7);
    stc.apply_to(mesh, Vector2{100, 0});
    object->geometries().create(mesh);

    auto& animator = scene.animator();
    animator.insert(*object,
                    [](Animation::UpdateInfo& info)
                    {
                        auto geo_slots = info.geo_slots();
                        auto geo = geo_slots[0]->geometry().as<SimplicialComplex>();

                        auto is_constrained =
                            geo->instances().find<IndexT>(builtin::is_constrained);
                        auto is_constrained_view = view(*is_constrained);
                        is_constrained_view[0]   = 1;

                        auto aim = geo->instances().find<Matrix4x4>(builtin::aim_transform);
                        auto            aim_view = view(*aim);
                        constexpr Float pi       = std::numbers::pi;
                        auto            theta    = info.frame() * 2 * pi / 360;
                        auto            sin_t    = std::sin(theta);

                        Transform t = Transform::Identity();
                        t.translate(Vector3{0, sin_t, 0});
                        aim_view[0] = t.matrix();
                    });


    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 360)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}