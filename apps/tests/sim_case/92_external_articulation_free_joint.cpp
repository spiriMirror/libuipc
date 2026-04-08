#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_free_joint.h>
#include <uipc/constitution/external_articulation_constraint.h>
#include <numbers>
#include <uipc/geometry/utils/affine_body/transform.h>

TEST_CASE("92_external_articulation_free_joint", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    Timer::enable_all();

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.4);
        SimplicialComplexIO io{pre_transform};

        auto body_object = scene.objects().create("body");

        AffineBodyConstitution abd;
        SimplicialComplex      abd_mesh =
            io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        label_surface(abd_mesh);
        abd.apply_to(abd_mesh, 100.0_MPa);

        {
            Transform t0 = Transform::Identity();
            t0.translate(Vector3{0, 1, 0});
            view(abd_mesh.transforms())[0] = t0.matrix();

            auto is_fixed = abd_mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 0;

            auto ref_dof_prev = abd_mesh.instances().create<Vector12>("ref_dof_prev");
            auto ref_dof_prev_view = view(*ref_dof_prev);
            auto transform_view    = abd_mesh.transforms().view();
            std::ranges::transform(transform_view,
                                   ref_dof_prev_view.begin(),
                                   affine_body::transform_to_q);

            auto external_kinetic =
                abd_mesh.instances().find<IndexT>(builtin::external_kinetic);
            view(*external_kinetic)[0] = 1;
        }
        auto [body_geo_slot, body_rest_geo_slot] =
            body_object->geometries().create(abd_mesh);

        scene.animator().insert(
            *body_object,
            [&](Animation::UpdateInfo& info)
            {
                auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();
                auto ref_dof_prev = geo->instances().find<Vector12>("ref_dof_prev");
                auto rdp_view       = view(*ref_dof_prev);
                auto transform_view = geo->transforms().view();
                std::ranges::transform(transform_view, rdp_view.begin(), affine_body::transform_to_q);
            });

        AffineBodyFreeJoint abfj;

        vector<S<SimplicialComplexSlot>> geo_slots    = {body_geo_slot};
        vector<IndexT>                   instance_ids = {0};
        auto free_joint_mesh = abfj.create_geometry(geo_slots, instance_ids);

        auto joint_object = scene.objects().create("free_joint");
        auto [free_joint_slot, free_joint_rest] =
            joint_object->geometries().create(free_joint_mesh);

        ExternalArticulationConstraint eac;
        {
            constexpr IndexT              n_dofs = 6;
            vector<S<const GeometrySlot>> joint_geos(n_dofs, free_joint_slot);
            vector<IndexT>                indices = {0, 1, 2, 3, 4, 5};
            auto articulation = eac.create_geometry(joint_geos, indices);

            auto mass      = articulation["joint_joint"]->find<Float>("mass");
            auto mass_view = view(*mass);
            Eigen::Map<MatrixX> mass_mat(mass_view.data(), n_dofs, n_dofs);
            mass_mat = MatrixX::Identity(n_dofs, n_dofs) * 64.0;

            auto articulation_object = scene.objects().create("articulation");
            articulation_object->geometries().create(articulation);

            scene.animator().insert(*articulation_object,
                                    [](Animation::UpdateInfo& info)
                                    {
                                        Float dt        = info.dt();
                                        auto  geo_slots = info.geo_slots();
                                        auto& geo = geo_slots[0]->geometry();

                                        auto delta_theta_tilde =
                                            geo["joint"]->find<Float>("delta_theta_tilde");
                                        auto dtv = view(*delta_theta_tilde);

                                        dtv[0] = 0.05 * dt;  // TransX
                                        dtv[1] = 0.0;        // TransY
                                        dtv[2] = 0.0;        // TransZ
                                        dtv[3] = 0.0;        // RotX
                                        dtv[4] = 0.0;        // RotY
                                        dtv[5] = std::numbers::pi / 12 * dt;  // RotZ
                                    });
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }

    Timer::report();
}
