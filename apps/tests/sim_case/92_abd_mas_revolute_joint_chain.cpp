#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>

// Correctness smoke test for ABDMASPreconditioner.
// A pendulum chain of 4 cubes connected by revolute joints (Z-axis).
// Body 0 is fixed; bodies 1-3 hang and swing freely under gravity.
// ABD MAS activates automatically because revolute joints are present.
TEST_CASE("92_abd_mas_revolute_joint_chain", "[abd][mas]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                            = test::Scene::default_config();
    config["gravity"]                      = Vector3{0, -9.8, 0};
    config["contact"]["enable"]            = false;
    config["line_search"]["report_energy"] = false;
    config["dt"]                           = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};

    AffineBodyConstitution abd;

    constexpr int N          = 4;       // chain length
    constexpr Float spacing  = 0.30;    // cubes face-to-face: center gap = 2×half_extent = 2×0.15
    constexpr Float stiffness = 100.0_MPa;

    // Create N bodies stacked vertically; body 0 is fixed at top.
    vector<S<SimplicialComplexSlot>> body_slots(N);

    for(int i = 0; i < N; i++)
    {
        auto obj  = scene.objects().create(fmt::format("body_{}", i));
        auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        abd.apply_to(mesh, stiffness);
        label_surface(mesh);

        Transform t = Transform::Identity();
        t.translate(Vector3{i * spacing, 0, 0});
        view(mesh.transforms())[0] = t.matrix();

        if(i == 0)
        {
            auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;
        }

        auto [slot, _] = obj->geometries().create(mesh);
        body_slots[i]  = slot;
    }

    // Connect consecutive bodies with revolute joints (axis along Z).
    // Local attachment point on each body: right face of left body / left face of right body.
    // Axis endpoints at (±0.15, 0, ±0.15) in local body space (cube scaled to 0.3).
    AffineBodyRevoluteJoint revolute_joint;

    vector<Vector3>                  l_pos0, l_pos1, r_pos0, r_pos1;
    vector<S<SimplicialComplexSlot>> l_slots, r_slots;
    vector<IndexT>                   l_ids, r_ids;
    vector<Float>                    strengths;

    for(int i = 0; i < N - 1; i++)
    {
        // Attachment axis along Z at the shared face: right face of body i, left face of body i+1
        l_pos0.push_back(Vector3{ 0.15, 0.0, -0.15});
        l_pos1.push_back(Vector3{ 0.15, 0.0,  0.15});
        r_pos0.push_back(Vector3{-0.15, 0.0, -0.15});
        r_pos1.push_back(Vector3{-0.15, 0.0,  0.15});
        l_slots.push_back(body_slots[i]);
        r_slots.push_back(body_slots[i + 1]);
        l_ids.push_back(0);
        r_ids.push_back(0);
        strengths.push_back(100.0);
    }

    auto joint_mesh = revolute_joint.create_geometry(span{l_pos0},
                                                      span{l_pos1},
                                                      span{r_pos0},
                                                      span{r_pos1},
                                                      span{l_slots},
                                                      span{l_ids},
                                                      span{r_slots},
                                                      span{r_ids},
                                                      span{strengths});

    auto joint_obj = scene.objects().create("revolute_joints");
    joint_obj->geometries().create(joint_mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
