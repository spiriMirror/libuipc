#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/affine_body_free_joint.h>
#include <uipc/constitution/external_articulation_constraint.h>
#include <numbers>
#include <uipc/geometry/utils/affine_body/transform.h>

TEST_CASE("93_external_articulation_free_joint_multijoints", "[abd]")
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

        auto links = scene.objects().create("links");

        AffineBodyConstitution abd;
        SimplicialComplex      abd_mesh =
            io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        abd_mesh.instances().resize(3);
        label_surface(abd_mesh);
        abd.apply_to(abd_mesh, 100.0_MPa);

        Transform t0 = Transform::Identity();
        t0.translate(Vector3::UnitZ() * -0.8);
        view(abd_mesh.transforms())[0] = t0.matrix();

        Transform t1 = Transform::Identity();
        t1.translate(Vector3::UnitZ() * 0.0);
        view(abd_mesh.transforms())[1] = t1.matrix();

        Transform t2 = Transform::Identity();
        t2.translate(Vector3::UnitZ() * 0.8);
        view(abd_mesh.transforms())[2] = t2.matrix();

        auto is_fixed = abd_mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);
        is_fixed_view[0] = 0;  // instance 0 NOT fixed (controlled by FreeJoint)
        is_fixed_view[1] = 0;
        is_fixed_view[2] = 0;

        auto ref_dof_prev = abd_mesh.instances().create<Vector12>("ref_dof_prev");
        auto ref_dof_prev_view = view(*ref_dof_prev);
        auto transform_view    = abd_mesh.transforms().view();
        std::ranges::transform(transform_view, ref_dof_prev_view.begin(), affine_body::transform_to_q);

        auto external_kinetic = abd_mesh.instances().find<IndexT>(builtin::external_kinetic);
        auto external_kinetic_view = view(*external_kinetic);
        std::ranges::fill(external_kinetic_view, 1);

        auto [geo_slot, rest_geo_slot] = links->geometries().create(abd_mesh);

        scene.animator().insert(
            *links,
            [&](Animation::UpdateInfo& info)
            {
                auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();
                auto ref_dof_prev = geo->instances().find<Vector12>("ref_dof_prev");
                auto rdp_view       = view(*ref_dof_prev);
                auto transform_view = geo->transforms().view();
                std::ranges::transform(transform_view, rdp_view.begin(), affine_body::transform_to_q);
            });

        // FreeJoint on instance 0 (replaces the fixed constraint in test 47)
        S<SimplicialComplexSlot> free_joint_slot;
        {
            AffineBodyFreeJoint abfj;

            vector<S<SimplicialComplexSlot>> fj_geo_slots    = {geo_slot};
            vector<IndexT>                   fj_instance_ids = {0};
            auto free_joint_mesh = abfj.create_geometry(fj_geo_slots, fj_instance_ids);

            auto fj_object = scene.objects().create("free_joint");
            auto [fj_slot, fj_rest_slot] = fj_object->geometries().create(free_joint_mesh);
            free_joint_slot = fj_slot;
        }

        // Revolute joint connecting instance 0 -> instance 1
        S<SimplicialComplexSlot> revolute_slot;
        {
            AffineBodyRevoluteJoint abrj;

            vector<Vector3>                  pos0s       = {{-0.5, 0.0, -0.4}};
            vector<Vector3>                  pos1s       = {{0.5, 0.0, -0.4}};
            vector<S<SimplicialComplexSlot>> l_geo_slots = {geo_slot};
            vector<IndexT>                   l_instance_ids  = {0};
            vector<S<SimplicialComplexSlot>> r_geo_slots     = {geo_slot};
            vector<IndexT>                   r_instance_ids  = {1};
            vector<Float>                    strength_ratios = {100.0};

            auto joint_mesh = abrj.create_geometry(
                pos0s, pos1s, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);

            auto joints = scene.objects().create("joints_revolute");
            auto [rj_slot, rj_rest_slot] = joints->geometries().create(joint_mesh);
            revolute_slot = rj_slot;
        }

        // Prismatic joint connecting instance 1 -> instance 2
        S<SimplicialComplexSlot> prismatic_slot;
        {
            AffineBodyPrismaticJoint abpj;

            vector<Vector3>                  pos0s          = {{0.0, 0.0, 0.0}};
            vector<Vector3>                  pos1s          = {{0.0, 0.0, 0.4}};
            vector<S<SimplicialComplexSlot>> l_geo_slots    = {geo_slot};
            vector<IndexT>                   l_instance_ids = {1};
            vector<S<SimplicialComplexSlot>> r_geo_slots    = {geo_slot};
            vector<IndexT>                   r_instance_ids = {2};
            vector<Float>                    strength_ratios = {100.0};

            auto joint_mesh = abpj.create_geometry(
                pos0s, pos1s, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);

            auto joints = scene.objects().create("joints_prismatic");
            auto [pj_slot, pj_rest_slot] = joints->geometries().create(joint_mesh);
            prismatic_slot = pj_slot;
        }

        // Articulation: 6 FreeJoint DOFs + 1 revolute + 1 prismatic = 8 joints
        ExternalArticulationConstraint eac;
        {
            constexpr IndexT n_free_dofs = 6;
            constexpr IndexT n_total = n_free_dofs + 2;  // 6 free + 1 revolute + 1 prismatic

            vector<S<const GeometrySlot>> joint_geos;
            vector<IndexT>                indices;

            for(IndexT i = 0; i < n_free_dofs; ++i)
            {
                joint_geos.push_back(free_joint_slot);
                indices.push_back(i);
            }
            joint_geos.push_back(revolute_slot);
            indices.push_back(0);
            joint_geos.push_back(prismatic_slot);
            indices.push_back(0);

            auto articulation = eac.create_geometry(joint_geos, indices);
            auto mass = articulation["joint_joint"]->find<Float>("mass");
            REQUIRE(mass);
            auto                mass_view = view(*mass);
            Eigen::Map<MatrixX> mass_mat(mass_view.data(), n_total, n_total);
            mass_mat       = MatrixX::Identity(n_total, n_total) * 64;
            mass_mat(6, 7) = 8;
            mass_mat(7, 6) = 8;

            auto articulation_object = scene.objects().create("articulation_object");
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

                                        dtv[0] = 0.1 * dt;  // TransX
                                        dtv[1] = 0.0;       // TransY
                                        dtv[2] = 0.0;       // TransZ
                                        dtv[3] = 0.0;       // RotX
                                        dtv[4] = 0.0;       // RotY
                                        dtv[5] = std::numbers::pi / 3 * dt;  // RotZ
                                        dtv[6] = std::numbers::pi / 6 * dt;  // Revolute
                                        dtv[7] = 0.1 * dt;  // Prismatic
                                    });
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 120)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }

    Timer::report();
}
