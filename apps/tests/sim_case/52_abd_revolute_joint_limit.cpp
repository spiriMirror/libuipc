#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_revolute_joint_limit.h>

TEST_CASE("52_abd_revolute_joint_limit", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"] = true;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    scene.contact_tabular().default_model(0.5, 1.0_GPa);

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.4);
    SimplicialComplexIO io{pre_transform};

    auto left_link  = scene.objects().create("left");
    auto right_link = scene.objects().create("right");

    AffineBodyConstitution abd;
    SimplicialComplex abd_mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(abd_mesh, 100.0_MPa);
    label_surface(abd_mesh);

    SimplicialComplex left_mesh = abd_mesh;
    {
        left_mesh.instances().resize(1);
        Transform t = Transform::Identity();
        t.translate(Vector3::UnitX() * -0.6);
        view(left_mesh.transforms())[0] = t.matrix();
        auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 0;
    }
    auto [left_geo_slot, left_rest_geo_slot] = left_link->geometries().create(left_mesh);

    SimplicialComplex right_mesh = abd_mesh;
    {
        right_mesh.instances().resize(1);
        Transform t = Transform::Identity();
        t.translate(Vector3::UnitX() * 0.6);
        view(right_mesh.transforms())[0] = t.matrix();
        auto is_fixed = right_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;
    }
    auto [right_geo_slot, right_rest_geo_slot] = right_link->geometries().create(right_mesh);

    vector<Vector2i> Es = {{0, 1}, {2, 3}};
    vector<Vector3> Vs = {
        Vector3{0.0, 0.0, -0.3},
        Vector3{0.0, 0.0, 0.3},
        Vector3{0.2, 0.0, -0.3},
        Vector3{0.2, 0.0, 0.3},
    };
    auto joint_mesh = linemesh(Vs, Es);
    label_surface(joint_mesh);

    AffineBodyRevoluteJoint revolute_joint;
    {
        vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot, left_geo_slot};
        vector<IndexT> l_instance_id                 = {0, 0};
        vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot, right_geo_slot};
        vector<IndexT> r_instance_id                 = {0, 0};
        vector<Float>  strength_ratios               = {100.0, 100.0};

        revolute_joint.apply_to(joint_mesh,
                                span{l_geo_slots},
                                span{l_instance_id},
                                span{r_geo_slots},
                                span{r_instance_id},
                                span{strength_ratios});
    }

    AffineBodyRevoluteJointLimit revolute_limit;
    vector<Float>                lowers    = {-0.25f, 0.3f};
    vector<Float>                uppers    = {0.25f, 0.3f};
    vector<Float>                strengths = {50.0f, 50.0f};
    revolute_limit.apply_to(joint_mesh, span{lowers}, span{uppers}, span{strengths});

    auto joints = scene.objects().create("revolute_joint");
    joints->geometries().create(joint_mesh);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));
    
    world.init(scene);
    REQUIRE(world.is_valid());

    while(world.frame() < 60)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
