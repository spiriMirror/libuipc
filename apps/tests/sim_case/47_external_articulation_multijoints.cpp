#include <app/catch2.h>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/external_articulation_constraint.h>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <uipc/geometry/utils/affine_body/transform.h>

TEST_CASE("47_external_articulation_multijoints", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    Timer::enable_all();

    auto this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                 = Scene::default_config();
    config["gravity"]           = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.4);
        SimplicialComplexIO io{pre_transform};

        // create object
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
        is_fixed_view[0]   = 1;  // instance 0 fixed
        is_fixed_view[1]   = 0;  // instance 1 not fixed
        is_fixed_view[2]   = 0;  // instance 2 not fixed

        auto ref_dof_prev = abd_mesh.instances().create<Vector12>("ref_dof_prev");
        auto ref_dof_prev_view = view(*ref_dof_prev);
        auto transform_view    = abd_mesh.transforms().view();
        std::ranges::transform(transform_view, ref_dof_prev_view.begin(), affine_body::transform_to_q);

        auto external_kinetic = abd_mesh.instances().find<IndexT>(builtin::external_kinetic);
        auto external_kinetic_view = view(*external_kinetic);
        std::ranges::fill(external_kinetic_view, 1);  // enable external kinetic for all instances

        auto [geo_slot, rest_geo_slot] = links->geometries().create(abd_mesh);

        scene.animator().insert(
            *links,
            [&](Animation::UpdateInfo& info)
            {
                auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();
                auto ref_dof_prev = geo->instances().find<Vector12>("ref_dof_prev");
                auto ref_dof_prev_view = view(*ref_dof_prev);
                auto transform_view    = geo->transforms().view();
                std::ranges::transform(transform_view,
                                       ref_dof_prev_view.begin(),
                                       affine_body::transform_to_q);
            });

        S<SimplicialComplexSlot> revolute_slot;
        {
            AffineBodyRevoluteJoint abrj;

            // Each edge defines a joint axis (2 points per edge)
            vector<Vector2i> Es         = {{0, 1}};
            vector<Vector3>  Vs         = {{-0.5, 0.0, -0.4}, {0.5, 0.0, -0.4}};
            auto             joint_mesh = linemesh(Vs, Es);
            // label_surface(joint_mesh);  // visualize the joint

            // Use multi-instance API:
            vector<S<SimplicialComplexSlot>> l_geo_slots     = {geo_slot};
            vector<IndexT>                   l_instance_id   = {0};
            vector<S<SimplicialComplexSlot>> r_geo_slots     = {geo_slot};
            vector<IndexT>                   r_instance_id   = {1};
            vector<Float>                    strength_ratios = {100.0};

            abrj.apply_to(joint_mesh, l_geo_slots, l_instance_id, r_geo_slots, r_instance_id, strength_ratios);

            auto joints = scene.objects().create("joints");
            auto [revolute_joint_slots, rest_revolute_joint_slots] =
                joints->geometries().create(joint_mesh);
            revolute_slot = revolute_joint_slots;
        }

        S<SimplicialComplexSlot> prismatic_slot;
        {
            AffineBodyPrismaticJoint abpj;
            // Each edge defines a joint axis (2 points per edge)
            vector<Vector2i> Es         = {{0, 1}};
            vector<Vector3>  Vs         = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.4}};
            auto             joint_mesh = linemesh(Vs, Es);
            // label_surface(joint_mesh);  // visualize the joint

            // Use multi-instance API:
            vector<S<SimplicialComplexSlot>> l_geo_slots     = {geo_slot};
            vector<IndexT>                   l_instance_id   = {1};
            vector<S<SimplicialComplexSlot>> r_geo_slots     = {geo_slot};
            vector<IndexT>                   r_instance_id   = {2};
            vector<Float>                    strength_ratios = {100.0};
            abpj.apply_to(joint_mesh, l_geo_slots, l_instance_id, r_geo_slots, r_instance_id, strength_ratios);
            auto joints = scene.objects().create("joints_prismatic");
            auto [prismatic_joint_slots, rest_prismatic_joint_slots] =
                joints->geometries().create(joint_mesh);
            prismatic_slot = prismatic_joint_slots;
        }

        ExternalArticulationConstraint eac;
        {
            vector<S<const GeometrySlot>> joint_geos = {revolute_slot, prismatic_slot};
            vector<IndexT> indices = {0, 0};
            auto articulation      = eac.create_geometry(joint_geos, indices);
            auto mass = articulation["joint_joint"]->find<Float>("mass");
            REQUIRE(mass);
            auto mass_view = view(*mass);
            Eigen::Map<MatrixX> mass_mat = Eigen::Map<MatrixX>(mass_view.data(), 2, 2);
            mass_mat       = MatrixX::Identity(2, 2) * 1e4;
            mass_mat(0, 1) = 5e3;
            mass_mat(1, 0) = 5e3;

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

                                        auto delta_theta_view = view(*delta_theta_tilde);
                                        delta_theta_view[0] = std::numbers::pi / 6 * dt;
                                        delta_theta_view[1] = 0.1 * dt;
                                    });
        }
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(
        fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    REQUIRE(world.is_valid());

    while(world.frame() < 240)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }

    Timer::report();
}