#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>
#include <affine_body/constitutions/joint_limit_penalty.h>
#include <affine_body/constraints/external_articulation_constraint_function.h>
#include <time_integrator/time_integrator.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/offset_count_collection.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/enumerate.h>
#include <affine_body/utils.h>
#include <affine_body/affine_body_external_force_reporter.h>
#include <joint_dof_system/joint_dof_reporter.h>
#include <muda/ext/eigen/atomic.h>
#include <numbers>

namespace uipc::backend::cuda
{
class AffineBodyPrismaticJoint final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 20;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    using InterAffineBodyConstitution::InterAffineBodyConstitution;


    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    // [    body0   |   body1    ]
    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6>  rest_cs;  // c_bar
    muda::DeviceBuffer<Vector6>  rest_ts;  // t_bar
    muda::DeviceBuffer<Vector6>  rest_ns;  // n_bar
    muda::DeviceBuffer<Vector6>  rest_bs;  // b_bar
    muda::DeviceBuffer<Float>    strength_ratios;

    vector<Vector2i> h_body_ids;
    vector<Vector6>  h_rest_cs;
    vector<Vector6>  h_rest_ts;
    vector<Vector6>  h_rest_ns;
    vector<Vector6>  h_rest_bs;
    vector<Float>    h_strength_ratios;

    // Distance tracking (for all prismatic joints)
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;
    vector<Float>                 h_init_distances;
    vector<Float>                 h_current_distances;

    muda::DeviceBuffer<Float> init_distances;
    muda::DeviceBuffer<Float> current_distances;

    BufferDump curr_distances_dump;

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;

    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        on_write_scene([this]() { write_scene(); });
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        h_geo_joint_offsets_counts.resize(info.inter_geo_infos().size());

        list<Vector2i> body_ids_list;
        list<Vector6>  rest_c_list;
        list<Vector6>  rest_t_list;
        list<Vector6>  rest_n_list;
        list<Vector6>  rest_b_list;
        list<Float>    strength_ratio_list;
        list<Float>    init_distances_list;

        IndexT geo_index = 0;
        info.for_each(
            geo_slots,
            [&](geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                U64  uid_value = uid->view()[0];
                UIPC_ASSERT(uid_value == ConstitutionUID,
                            "AffineBodyPrismaticJoint: Geometry constitution UID mismatch");

                auto sc = geo.as<geometry::SimplicialComplex>();

                h_geo_joint_offsets_counts.counts()[geo_index] = sc->edges().size();

                auto l_geo_id = sc->edges().find<IndexT>("l_geo_id");
                UIPC_ASSERT(l_geo_id, "AffineBodyPrismaticJoint: Geometry must have 'l_geo_id' attribute on `edges`");
                auto l_geo_id_view = l_geo_id->view();

                auto r_geo_id = sc->edges().find<IndexT>("r_geo_id");
                UIPC_ASSERT(r_geo_id, "AffineBodyPrismaticJoint: Geometry must have 'r_geo_id' attribute on `edges`");
                auto r_geo_id_view = r_geo_id->view();

                auto l_inst_id = sc->edges().find<IndexT>("l_inst_id");
                UIPC_ASSERT(l_inst_id, "AffineBodyPrismaticJoint: Geometry must have 'l_inst_id' attribute on `edges`");
                auto l_inst_id_view = l_inst_id->view();

                auto r_inst_id = sc->edges().find<IndexT>("r_inst_id");
                UIPC_ASSERT(r_inst_id, "AffineBodyPrismaticJoint: Geometry must have 'r_inst_id' attribute on `edges`");
                auto r_inst_id_view = r_inst_id->view();

                auto strength_ratio = sc->edges().find<Float>("strength_ratio");
                UIPC_ASSERT(strength_ratio, "AffineBodyPrismaticJoint: Geometry must have 'strength_ratio' attribute on `edges`");
                auto strength_ratio_view = strength_ratio->view();

                auto init_distance_attr = sc->edges().find<Float>("init_distance");
                UIPC_ASSERT(init_distance_attr,
                            "AffineBodyPrismaticJoint: Geometry must have 'init_distance' attribute on `edges`");
                auto init_distance_view = init_distance_attr->view();

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();

                auto l_pos0_attr = sc->edges().find<Vector3>("l_position0");
                auto l_pos1_attr = sc->edges().find<Vector3>("l_position1");
                auto r_pos0_attr = sc->edges().find<Vector3>("r_position0");
                auto r_pos1_attr = sc->edges().find<Vector3>("r_position1");
                bool use_local = l_pos0_attr && l_pos1_attr && r_pos0_attr && r_pos1_attr;

                for(auto&& [i, e] : enumerate(Es))
                {
                    IndexT l_gid = l_geo_id_view[i];
                    IndexT r_gid = r_geo_id_view[i];
                    IndexT l_iid = l_inst_id_view[i];
                    IndexT r_iid = r_inst_id_view[i];

                    Vector2i body_ids = {info.body_id(l_gid, l_iid),
                                         info.body_id(r_gid, r_iid)};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, l_gid);
                    auto right_sc = info.body_geo(geo_slots, r_gid);

                    UIPC_ASSERT(l_iid >= 0
                                    && l_iid < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Left instance ID {} is out of range [0, {})",
                                l_iid,
                                left_sc->instances().size());
                    UIPC_ASSERT(r_iid >= 0
                                    && r_iid < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Right instance ID {} is out of range [0, {})",
                                r_iid,
                                right_sc->instances().size());

                    Transform LT{left_sc->transforms().view()[l_iid]};
                    Transform RT{right_sc->transforms().view()[r_iid]};

                    Vector3 t;
                    Vector6 rest_position_c;
                    if(use_local)
                    {
                        Vector3 lp0 = LT * l_pos0_attr->view()[i];
                        Vector3 lp1 = LT * l_pos1_attr->view()[i];
                        t           = (lp1 - lp0).normalized();
                        rest_position_c.segment<3>(0) = l_pos0_attr->view()[i];
                        rest_position_c.segment<3>(3) = r_pos0_attr->view()[i];
                    }
                    else
                    {
                        Vector3 P0 = Ps[e[0]];
                        Vector3 P1 = Ps[e[1]];
                        UIPC_ASSERT((P0 - P1).squaredNorm() > 0,
                                    "AffineBodyPrismaticJoint: Edge positions must not be too close");
                        t                             = (P1 - P0).normalized();
                        rest_position_c.segment<3>(0) = LT.inverse() * P0;
                        rest_position_c.segment<3>(3) = RT.inverse() * P0;
                    }

                    Vector3 n, b;
                    orthonormal_basis(t, n, b);

                    rest_c_list.push_back(rest_position_c);

                    Matrix3x3 LT_rotation = LT.rotation();
                    Matrix3x3 RT_rotation = RT.rotation();

                    Vector6 rest_vec_t;
                    rest_vec_t.segment<3>(0) = LT_rotation.inverse() * t;
                    rest_vec_t.segment<3>(3) = RT_rotation.inverse() * t;
                    rest_t_list.push_back(rest_vec_t);

                    Vector6 rest_vec_n;
                    rest_vec_n.segment<3>(0) = LT_rotation.inverse() * n;
                    rest_vec_n.segment<3>(3) = RT_rotation.inverse() * n;
                    rest_n_list.push_back(rest_vec_n);

                    Vector6 rest_vec_b;
                    rest_vec_b.segment<3>(0) = LT_rotation.inverse() * b;
                    rest_vec_b.segment<3>(3) = RT_rotation.inverse() * b;
                    rest_b_list.push_back(rest_vec_b);

                    init_distances_list.push_back(init_distance_view[i]);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
                ++geo_index;
            });

        h_geo_joint_offsets_counts.scan();

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_rest_cs.resize(rest_c_list.size());
        std::ranges::move(rest_c_list, h_rest_cs.begin());

        h_rest_ts.resize(rest_t_list.size());
        std::ranges::move(rest_t_list, h_rest_ts.begin());

        h_rest_ns.resize(rest_n_list.size());
        std::ranges::move(rest_n_list, h_rest_ns.begin());

        h_rest_bs.resize(rest_b_list.size());
        std::ranges::move(rest_b_list, h_rest_bs.begin());

        h_strength_ratios.resize(strength_ratio_list.size());
        std::ranges::move(strength_ratio_list, h_strength_ratios.begin());

        h_init_distances.resize(init_distances_list.size());
        std::ranges::copy(init_distances_list, h_init_distances.begin());

        h_current_distances.resize(h_init_distances.size());

        body_ids.copy_from(h_body_ids);
        rest_cs.copy_from(h_rest_cs);
        rest_ts.copy_from(h_rest_ts);
        rest_ns.copy_from(h_rest_ns);
        rest_bs.copy_from(h_rest_bs);
        strength_ratios.copy_from(h_strength_ratios);
        init_distances.copy_from(h_init_distances);
        current_distances.copy_from(h_current_distances);

        compute_current_distances();
        write_scene();
    }

    void compute_current_distances()
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_cs  = rest_cs.cviewer().name("rest_cs"),
                    rest_ts  = rest_ts.cviewer().name("rest_ts"),
                    qs       = affine_body_dynamics->qs().cviewer().name("qs"),
                    current_distances = current_distances.viewer().name("current_distances"),
                    init_distances = init_distances.cviewer().name("init_distances")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       const Vector6& C_bar = rest_cs(I);
                       const Vector6& t_bar = rest_ts(I);

                       Float distance;
                       compute_absolute_distance(distance, C_bar, t_bar, q_i, q_j);

                       current_distances(I) = distance + init_distances(I);
                   });
    }

    void write_scene()
    {
        auto geo_slots = world().scene().geometries();

        current_distances.copy_to(h_current_distances);

        IndexT geo_joint_index = 0;

        this->for_each(
            geo_slots,
            [&](geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyPrismaticJoint: Geometry must be a simplicial complex");

                auto distance = sc->edges().find<Float>("distance");

                if(distance)
                {
                    auto distance_view = view(*distance);
                    auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];
                    UIPC_ASSERT(distance_view.size() == count,
                                "AffineBodyPrismaticJoint: distance attribute size {} mismatch with joint count {}",
                                distance_view.size(),
                                count);

                    auto src = span{h_current_distances}.subspan(offset, count);
                    std::ranges::copy(src, distance_view.begin());
                }

                ++geo_joint_index;
            });
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());  // one energy per joint
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace PJ = sym::affine_body_prismatic_joint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_cs  = rest_cs.cviewer().name("rest_cs"),
                    rest_ts  = rest_ts.cviewer().name("rest_ts"),
                    rest_ns  = rest_ns.cviewer().name("rest_ns"),
                    rest_bs  = rest_bs.cviewer().name("rest_bs"),
                    strength_ratio = strength_ratios.cviewer().name("strength_ratio"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    qs = info.qs().viewer().name("qs"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Float kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 qi = qs(bids(0));
                       Vector12 qj = qs(bids(1));

                       auto& rest_c = rest_cs(I);
                       auto& rest_t = rest_ts(I);
                       auto& rest_n = rest_ns(I);
                       auto& rest_b = rest_bs(I);

                       // Create Frame F01
                       Vector9 F01;
                       PJ::F01<Float>(F01,
                                      rest_c.segment<3>(0),  // ci_bar
                                      rest_t.segment<3>(0),  // ti_bar
                                      qi,                    // qi
                                      rest_c.segment<3>(3),  // cj_bar
                                      rest_t.segment<3>(3),  // tj_bar
                                      qj                     // qj
                       );

                       Float E01;
                       PJ::E01(E01, kappa, F01);

                       Float E23;
                       PJ::E23<Float>(E23,
                                      kappa,
                                      rest_n.segment<3>(0),  // ni_bar
                                      rest_b.segment<3>(0),  // bi_bar
                                      qi,                    // qi
                                      rest_n.segment<3>(3),  // nj_bar
                                      rest_b.segment<3>(3),  // bj_bar
                                      qj                     // qj
                       );

                       Es(I) = E01 + E23;
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(2 * body_ids.size());  // each joint has 2 * Vector12 gradients

        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * body_ids.size());  // each joint has HalfHessianSize * Matrix12x12 hessians
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace PJ       = sym::affine_body_prismatic_joint;
        auto gradient_only = info.gradient_only();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_cs  = rest_cs.cviewer().name("rest_cs"),
                    rest_ts  = rest_ts.cviewer().name("rest_ts"),
                    rest_ns  = rest_ns.cviewer().name("rest_ns"),
                    rest_bs  = rest_bs.cviewer().name("rest_bs"),
                    strength_ratio = strength_ratios.cviewer().name("strength_ratio"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    qs      = info.qs().viewer().name("qs"),
                    G12s    = info.gradients().viewer().name("G12s"),
                    H12x12s = info.hessians().viewer().name("H12x12s"),
                    gradient_only] __device__(int I) mutable
                   {
                       Vector2i bids = body_ids(I);

                       Float kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 qi = qs(bids(0));
                       Vector12 qj = qs(bids(1));

                       auto& rest_c = rest_cs(I);
                       auto& rest_t = rest_ts(I);
                       auto& rest_n = rest_ns(I);
                       auto& rest_b = rest_bs(I);

                       // Create Frame F01
                       Vector9 F01;
                       PJ::F01<Float>(F01,
                                      rest_c.segment<3>(0),  // ci_bar
                                      rest_t.segment<3>(0),  // ti_bar
                                      qi,                    // qi
                                      rest_c.segment<3>(3),  // cj_bar
                                      rest_t.segment<3>(3),  // tj_bar
                                      qj                     // qj
                       );

                       // Get Gradient based on Frame F01
                       Vector9 G01;
                       PJ::dE01dF01(G01, kappa, F01);
                       Vector24 J01T_G01;
                       PJ::J01T_G01<Float>(J01T_G01,
                                           G01,                   // G01
                                           rest_c.segment<3>(0),  // ci_bar
                                           rest_t.segment<3>(0),  // ti_bar
                                           rest_c.segment<3>(3),  // cj_bar
                                           rest_t.segment<3>(3)   // tj_bar
                       );

                       // Get Gradient based on [qi,qj] directly
                       Vector24 dE23dQ;
                       PJ::dE23dQ<Float>(dE23dQ,
                                         kappa,
                                         rest_n.segment<3>(0),  // ni_bar
                                         rest_b.segment<3>(0),  // bi_bar
                                         qi,                    // qi
                                         rest_n.segment<3>(3),  // nj_bar
                                         rest_b.segment<3>(3),  // bj_bar
                                         qj                     // qj
                       );

                       Vector24 G = J01T_G01 + dE23dQ;

                       DoubletVectorAssembler DVA{G12s};
                       Vector2i               indices = {bids(0), bids(1)};
                       DVA.segment<2>(2 * I).write(indices, G);

                       if(gradient_only)
                           return;

                       // Get Hessian based on Frame F01
                       Matrix9x9 H01;
                       PJ::ddE01ddF01(H01, kappa, F01);

                       // Ensure H01 is SPD
                       make_spd(H01);

                       Matrix24x24 J01T_H01_J01;
                       PJ::J01T_H01_J01<Float>(J01T_H01_J01,
                                               H01,
                                               rest_c.segment<3>(0),   // ci_bar
                                               rest_t.segment<3>(0),   // ti_bar
                                               rest_c.segment<3>(3),   // cj_bar
                                               rest_t.segment<3>(3));  // tj_bar

                       // Get Hessian based on [qi,qj] directly
                       Matrix24x24 ddE23ddQ;
                       PJ::ddE23ddQ<Float>(ddE23ddQ,
                                           kappa,
                                           rest_n.segment<3>(0),  // ni_bar
                                           rest_b.segment<3>(0),  // bi_bar
                                           qi,                    // qi
                                           rest_n.segment<3>(3),  // nj_bar
                                           rest_b.segment<3>(3),  // bj_bar
                                           qj                     // qj
                       );
                       Matrix24x24 H = J01T_H01_J01 + ddE23ddQ;

                       TripletMatrixAssembler TMA{H12x12s};
                       TMA.half_block<2>(HalfHessianSize * I).write(indices, H);
                   });
    };

    bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_distances_dump.dump(fmt::format("{}pj_current_distances.{}", path, frame),
                                        current_distances);
    }

    bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_distances_dump.load(fmt::format("{}pj_current_distances.{}", path, frame));
    }

    void do_apply_recover(RecoverInfo& info) override
    {
        curr_distances_dump.apply_to(current_distances);
    }

    void do_clear_recover(RecoverInfo& info) override
    {
        curr_distances_dump.clean_up();
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJoint);

class AffineBodyPrismaticJointDofReporter final : public JointDofReporter
{
  public:
    using JointDofReporter::JointDofReporter;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;

    void do_build(BuildInfo& info) override
    {
        prismatic_joint = require<AffineBodyPrismaticJoint>();
    }

    void do_update_dof_attributes(UpdateDofAttributesInfo& info) override
    {
        // Re-sync persistent per-joint distances (current_distances) with the
        // new vertex/body attribute layout before the next frame uses them.
        prismatic_joint->compute_current_distances();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointDofReporter);

class AffineBodyPrismaticJointTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;
    SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        prismatic_joint      = require<AffineBodyPrismaticJoint>();
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override {}

    void do_update_state(UpdateVelocityInfo& info) override
    {
        prismatic_joint->compute_current_distances();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointTimeIntegrator);

class AffineBodyDrivingPrismaticJoint : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    static constexpr SizeT StencilSize     = 2;

    static constexpr U64 ConstraintUID = 21;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;

    // Host
    // Note: body_ids / rest_cs / rest_ts / init_distances are reused directly
    // from AffineBodyPrismaticJoint under a strict index-alignment assumption
    // (see sanity check in do_init).
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;

    vector<IndexT> h_is_constrained;
    vector<Float>  h_strength_ratios;
    vector<IndexT> h_is_passive;
    vector<Float>  h_aim_distances;

    // Device
    muda::DeviceBuffer<IndexT> is_constrained;
    muda::DeviceBuffer<Float>  strength_ratios;
    muda::DeviceBuffer<IndexT> is_passive;
    muda::DeviceBuffer<Float>  aim_distances;

    void do_build(BuildInfo& info) override
    {
        prismatic_joint = require<AffineBodyPrismaticJoint>();
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();
        h_geo_joint_offsets_counts.resize(info.anim_inter_geo_infos().size());

        list<IndexT> is_constrained_list;
        list<Float>  strength_ratios_list;
        list<IndexT> is_passive_list;
        list<Float>  aim_distances_list;

        IndexT joint_offset = 0;
        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                // check uid
                {
                    auto constraint_uids =
                        geo.meta().find<VectorXu64>(builtin::constraint_uids);
                    UIPC_ASSERT(constraint_uids, "AffineBodyDrivingPrismaticJoint: Geometry must have 'constraint_uids' attribute");
                    bool has_this_constraint = false;
                    for(auto&& uid_value : constraint_uids->view().front())
                    {
                        if(uid_value == ConstraintUID)
                        {
                            has_this_constraint = true;
                            break;
                        }
                    }
                    UIPC_ASSERT(has_this_constraint,
                                "AffineBodyDrivingPrismaticJoint: Geometry must have constraint UID {}",
                                ConstraintUID);
                }
                // check constitution uid
                {
                    auto constitution_uid = geo.meta().find<U64>(builtin::constitution_uid);
                    UIPC_ASSERT(constitution_uid, "AffineBodyDrivingPrismaticJoint: Geometry must have 'constitution_uid' attribute");
                    U64 uid_value = constitution_uid->view().front();
                    UIPC_ASSERT(uid_value == AffineBodyPrismaticJoint::ConstitutionUID,
                                "AffineBodyDrivingPrismaticJoint: Geometry constitution UID mismatch");
                }
                // get simplicial complex
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyDrivingPrismaticJoint geometry must be SimplicialComplex");

                h_geo_joint_offsets_counts.counts()[joint_offset] = sc->edges().size();
                UIPC_ASSERT(h_geo_joint_offsets_counts.counts()[joint_offset] > 0,
                            "AffineBodyDrivingPrismaticJoint: Geometry must have at least one edge");

                // Driving-only attributes; shared attributes (body_ids, rest_cs,
                // rest_ts, init_distances) are supplied by AffineBodyPrismaticJoint.
                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                std::ranges::copy(is_constrained->view(),
                                  std::back_inserter(is_constrained_list));

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                std::ranges::copy(strength_ratios->view(),
                                  std::back_inserter(strength_ratios_list));

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingPrismaticJoint: Geometry must have 'is_passive' attribute on `edges`")
                std::ranges::copy(is_passive->view(), std::back_inserter(is_passive_list));

                auto aim_distances = sc->edges().find<Float>("aim_distance");
                UIPC_ASSERT(aim_distances, "AffineBodyDrivingPrismaticJoint: Geometry must have 'aim_distance' attribute on `edges`")
                std::ranges::copy(aim_distances->view(),
                                  std::back_inserter(aim_distances_list));

                joint_offset++;
            });
        // scan to compute offsets
        h_geo_joint_offsets_counts.scan();

        h_is_constrained.resize(is_constrained_list.size());
        std::ranges::copy(is_constrained_list, h_is_constrained.begin());

        h_strength_ratios.resize(strength_ratios_list.size());
        std::ranges::copy(strength_ratios_list, h_strength_ratios.begin());

        h_is_passive.resize(is_passive_list.size());
        std::ranges::copy(is_passive_list, h_is_passive.begin());

        h_aim_distances.resize(aim_distances_list.size());
        std::ranges::copy(aim_distances_list, h_aim_distances.begin());

        is_constrained.copy_from(h_is_constrained);
        strength_ratios.copy_from(h_strength_ratios);
        is_passive.copy_from(h_is_passive);
        aim_distances.copy_from(h_aim_distances);

        // Sanity check: driving joint reuses body_ids/rest_cs/rest_ts/init_distances
        // from AffineBodyPrismaticJoint via the shared index `I`. Both must
        // therefore enumerate exactly the same (geo, edge) sequence.
        UIPC_ASSERT(h_is_constrained.size() == prismatic_joint->h_body_ids.size(),
                    "AffineBodyDrivingPrismaticJoint: joint count {} must equal "
                    "AffineBodyPrismaticJoint joint count {} (index alignment required)",
                    h_is_constrained.size(),
                    prismatic_joint->h_body_ids.size());
    }


    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto  geo_slots       = world().scene().geometries();
        SizeT geo_joint_index = 0;

        // Mirror all per-edge driving attributes host-side, then upload each
        // buffer once at the end. No change-detection: attributes are expected
        // to change every step (they are the animation signal), so guarding
        // the uploads is pointless.
        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyDrivingPrismaticJoint: Geometry must be a simplicial complex");

                auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];

                UIPC_ASSERT(sc->edges().size() == count,
                            "AffineBodyDrivingPrismaticJoint: Geometry edges size {} mismatch with joint count {}",
                            sc->edges().size(),
                            count);

                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                std::ranges::copy(is_constrained->view(),
                                  span{h_is_constrained}.subspan(offset, count).begin());

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingPrismaticJoint: Geometry must have 'is_passive' attribute on `edges`")
                std::ranges::copy(is_passive->view(),
                                  span{h_is_passive}.subspan(offset, count).begin());

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                std::ranges::copy(strength_ratios->view(),
                                  span{h_strength_ratios}.subspan(offset, count).begin());

                auto aim_distances = sc->edges().find<Float>("aim_distance");
                UIPC_ASSERT(aim_distances, "AffineBodyDrivingPrismaticJoint: Geometry must have 'aim_distance' attribute on `edges`");
                std::ranges::copy(aim_distances->view(),
                                  span{h_aim_distances}.subspan(offset, count).begin());

                ++geo_joint_index;
            });

        aim_distances.copy_from(h_aim_distances);
        is_constrained.copy_from(h_is_constrained);
        strength_ratios.copy_from(h_strength_ratios);
        is_passive.copy_from(h_is_passive);
    }

    void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(is_constrained.size());
        info.gradient_count(2 * is_constrained.size());
        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * is_constrained.size());
    }


    void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace DPJ = sym::affine_body_driving_prismatic_joint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                is_constrained.size(),
                [body_ids = prismatic_joint->body_ids.cviewer().name("body_ids"),
                 is_constrained = is_constrained.cviewer().name("is_constrained"),
                 is_passive = is_passive.cviewer().name("is_passive"),
                 strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                 rest_positions = prismatic_joint->rest_cs.cviewer().name("rest_positions"),
                 rest_tangents = prismatic_joint->rest_ts.cviewer().name("rest_tangents"),
                 init_distances = prismatic_joint->init_distances.cviewer().name("init_distances"),
                 curr_distances = prismatic_joint->current_distances.cviewer().name("current_distances"),
                 aim_distances = aim_distances.cviewer().name("aim_distances"),
                 qs            = info.qs().cviewer().name("qs"),
                 body_masses = info.body_masses().cviewer().name("body_masses"),
                 is_fixed    = info.is_fixed().cviewer().name("is_fixed"),
                 Es = info.energies().viewer().name("Es")] __device__(int I)
                {
                    Vector2i bids        = body_ids(I);
                    auto     constrained = is_constrained(I);
                    if(constrained == 0)
                    {
                        Es(I) = 0.0;
                        return;
                    }
                    Float kappa =
                        strength_ratios(I)
                        * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

                    auto passive      = is_passive(I);
                    auto aim_distance = aim_distances(I);
                    if(passive == 1)
                    {
                        // resist external forces passively
                        aim_distance = curr_distances(I);
                    }

                    Float d_tidle = aim_distance - init_distances(I);

                    Vector12 q_i = qs(bids(0));
                    Vector12 q_j = qs(bids(1));

                    const Vector6& C_bar = rest_positions(I);
                    const Vector6& T_bar = rest_tangents(I);

                    Vector9 F01;
                    DPJ::F01_q<Float>(F01,
                                      C_bar.segment<3>(0),
                                      T_bar.segment<3>(0),
                                      q_i,
                                      C_bar.segment<3>(3),
                                      T_bar.segment<3>(3),
                                      q_j);

                    Float E01;
                    DPJ::E01<Float>(E01, kappa, F01, d_tidle);
                    Es(I) = E01;
                });
    }

    void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override
    {
        using namespace muda;
        namespace DPJ = sym::affine_body_driving_prismatic_joint;

        using Vector24    = Vector<Float, 24>;
        using Matrix24x24 = Matrix<Float, 24, 24>;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                is_constrained.size(),
                [body_ids = prismatic_joint->body_ids.cviewer().name("body_ids"),
                 is_constrained = is_constrained.cviewer().name("is_constrained"),
                 is_passive = is_passive.cviewer().name("is_passive"),
                 strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                 rest_positions = prismatic_joint->rest_cs.cviewer().name("rest_positions"),
                 rest_tangents = prismatic_joint->rest_ts.cviewer().name("rest_tangents"),
                 init_distances = prismatic_joint->init_distances.cviewer().name("init_distances"),
                 curr_distances = prismatic_joint->current_distances.cviewer().name("current_distances"),
                 aim_distances = aim_distances.cviewer().name("aim_distances"),
                 qs            = info.qs().cviewer().name("qs"),
                 body_masses = info.body_masses().cviewer().name("body_masses"),
                 is_fixed    = info.is_fixed().cviewer().name("is_fixed"),
                 G12s        = info.gradients().viewer().name("G12s"),
                 H12x12s = info.hessians().viewer().name("H12x12s")] __device__(int I)
                {
                    Vector2i bids = body_ids(I);

                    auto constrained = is_constrained(I);
                    if(constrained == 0)
                    {
                        DoubletVectorAssembler DVA{G12s};
                        DVA.segment<StencilSize>(I * StencilSize).write(bids, Vector24::Zero());

                        TripletMatrixAssembler TMA{H12x12s};
                        TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, Matrix24x24::Zero());
                        return;
                    }

                    Float kappa =
                        strength_ratios(I)
                        * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

                    auto passive      = is_passive(I);
                    auto aim_distance = aim_distances(I);
                    if(passive == 1)
                    {
                        // resist external forces passively
                        aim_distance = curr_distances(I);
                    }

                    Float d_tidle = aim_distance - init_distances(I);

                    Vector12 q_i = qs(bids(0));
                    Vector12 q_j = qs(bids(1));

                    const Vector6& C_bar = rest_positions(I);
                    const Vector6& t_bar = rest_tangents(I);

                    Vector9 F01_q;
                    DPJ::F01_q<Float>(F01_q,
                                      C_bar.segment<3>(0),
                                      t_bar.segment<3>(0),
                                      q_i,
                                      C_bar.segment<3>(3),
                                      t_bar.segment<3>(3),
                                      q_j);

                    // Gradient
                    Vector9 G01;
                    DPJ::dE01dF01(G01, kappa, F01_q, d_tidle);
                    Vector24 J01T_G01;
                    DPJ::J01T_G01<Float>(J01T_G01,
                                         G01,
                                         C_bar.segment<3>(0),
                                         t_bar.segment<3>(0),
                                         C_bar.segment<3>(3),
                                         t_bar.segment<3>(3));
                    DoubletVectorAssembler DVA{G12s};
                    DVA.segment<StencilSize>(StencilSize * I).write(bids, J01T_G01);

                    // Hessian
                    Matrix9x9 H01;
                    DPJ::ddE01ddF01(H01, kappa, F01_q, d_tidle);
                    make_spd(H01);
                    Matrix24x24 J01T_H01_J01;
                    DPJ::J01T_H01_J01<Float>(J01T_H01_J01,
                                             H01,
                                             C_bar.segment<3>(0),
                                             t_bar.segment<3>(0),
                                             C_bar.segment<3>(3),
                                             t_bar.segment<3>(3));
                    TripletMatrixAssembler TMA{H12x12s};
                    TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, J01T_H01_J01);
                });
    }

    U64 get_uid() const noexcept override { return ConstraintUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingPrismaticJoint);

// ============================================================================
// AffineBodyPrismaticJointExternalForceConstraint
// Reuses body_ids, rest_ts, init_distances from AffineBodyPrismaticJoint.
// Only stores external-force-specific data (forces, is_constrained).
// ============================================================================

class AffineBodyPrismaticJointExternalForceConstraint final : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;
    static constexpr U64 ConstraintUID = 667;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;

    vector<Float>  h_forces;
    vector<IndexT> h_is_constrained;

    muda::DeviceBuffer<Float>  forces;
    muda::DeviceBuffer<IndexT> is_constrained;

    void do_build(BuildInfo& info) override
    {
        prismatic_joint = require<AffineBodyPrismaticJoint>();
    }

    U64 get_uid() const noexcept override { return ConstraintUID; }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalForceConstraint: geometry must be SimplicialComplex");

                auto is_constrained =
                    sc->edges().find<IndexT>("external_force/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();

                auto external_force = sc->edges().find<Float>("external_force");
                UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force' attribute on `edges`");
                auto external_force_view = external_force->view();

                for(SizeT i = 0; i < sc->edges().size(); ++i)
                {
                    h_is_constrained.push_back(is_constrained_view[i]);
                    h_forces.push_back(external_force_view[i]);
                }
            });

        SizeT N = h_forces.size();
        if(N > 0)
        {
            forces.copy_from(h_forces);
            is_constrained.copy_from(h_is_constrained);
        }
    }

    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        SizeT offset = 0;
        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalForceConstraint: geometry must be SimplicialComplex");

                auto is_constrained =
                    sc->edges().find<IndexT>("external_force/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();

                auto external_force = sc->edges().find<Float>("external_force");
                UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force' attribute on `edges`");
                auto external_force_view = external_force->view();

                auto Es = sc->edges().topo().view();
                for(auto&& [i, e] : enumerate(Es))
                {
                    h_is_constrained[offset] = is_constrained_view[i];
                    h_forces[offset]         = external_force_view[i];
                    ++offset;
                }
            });

        SizeT N = h_forces.size();
        if(N > 0)
        {
            is_constrained.copy_from(h_is_constrained);
            forces.copy_from(h_forces);
        }
    }

    void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(0);
        info.gradient_count(0);
        if(!info.gradient_only())
            info.hessian_count(0);
    }

    void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override
    {
    }
    void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override
    {
    }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalForceConstraint);

// ============================================================================
// AffineBodyPrismaticJointExternalForce
// Applies external forces along prismatic joint axes.
// ============================================================================

class AffineBodyPrismaticJointExternalForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 ReporterUID = 667;
    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyPrismaticJointExternalForceConstraint> constraint;

    void do_build(BuildInfo& info) override
    {
        constraint = require<AffineBodyPrismaticJointExternalForceConstraint>();
    }

    U64 get_uid() const noexcept override { return ReporterUID; }

    void do_init() override {}

    void do_step(ExternalForceInfo& info) override
    {
        SizeT force_count = constraint->forces.size();
        if(force_count == 0)
            return;

        auto abd = constraint->prismatic_joint->affine_body_dynamics;

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(force_count,
                   [external_forces = info.external_forces().viewer().name("external_forces"),
                    body_ids = constraint->prismatic_joint->body_ids.cviewer().name("body_ids"),
                    forces = constraint->forces.cviewer().name("forces"),
                    rest_tangents =
                        constraint->prismatic_joint->rest_ts.cviewer().name("rest_tangents"),
                    constrained_flags = constraint->is_constrained.cviewer().name("constrained_flags"),
                    qs = abd->qs().cviewer().name("qs")] __device__(int i) mutable
                   {
                       if(constrained_flags(i) == 0)
                           return;

                       Vector2i bids = body_ids(i);
                       Float    f    = forces(i);

                       const Vector6& t_bar = rest_tangents(i);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       ABDJacobi JT[2] = {ABDJacobi{t_bar.segment<3>(0)},
                                          ABDJacobi{t_bar.segment<3>(3)}};
                       Vector3   t_i   = JT[0].vec_x(q_i);
                       Vector3   t_j   = JT[1].vec_x(q_j);

                       // symmetrize to avoid numerical issues when the joint is near singularity
                       t_j = 0.5 * (t_i + t_j);
                       t_i = -t_j;

                       // Build 12D force vectors: -f*t to body_i, +f*t to body_j
                       Vector12 F_i      = Vector12::Zero();
                       F_i.segment<3>(0) = f * t_i;

                       Vector12 F_j      = Vector12::Zero();
                       F_j.segment<3>(0) = f * t_j;

                       eigen::atomic_add(external_forces(bids(0)), F_i);
                       eigen::atomic_add(external_forces(bids(1)), F_j);
                   });
    }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalForce);

// ============================================================================
// AffineBodyPrismaticJointLimit
// Penalty energy enforcing `limit/lower <= delta_distance <= limit/upper`.
// Reuses body_ids / rest_cs / rest_ts / init_distances from
// AffineBodyPrismaticJoint (index-aligned). Only ref_qs, lowers, uppers,
// strengths are owned locally.
// ============================================================================

class AffineBodyPrismaticJointLimit final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 669;
    static constexpr U64   JointUID = AffineBodyPrismaticJoint::ConstitutionUID;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;

    vector<Vector24> h_ref_qs;
    vector<Float>    h_lowers;
    vector<Float>    h_uppers;
    vector<Float>    h_strengths;

    muda::DeviceBuffer<Vector24> ref_qs;
    muda::DeviceBuffer<Float>    lowers;
    muda::DeviceBuffer<Float>    uppers;
    muda::DeviceBuffer<Float>    strengths;

    void do_build(BuildInfo& info) override
    {
        prismatic_joint = require<AffineBodyPrismaticJoint>();
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        h_ref_qs.clear();
        h_lowers.clear();
        h_uppers.clear();
        h_strengths.clear();

        info.for_each(
            geo_slots,
            [&](geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                UIPC_ASSERT(uid && uid->view()[0] == JointUID,
                            "AffineBodyPrismaticJointLimit must be attached on base prismatic joint geometry (UID={})",
                            JointUID);

                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyPrismaticJointLimit geometry must be SimplicialComplex");

                auto l_geo_id = sc->edges().find<IndexT>("l_geo_id");
                UIPC_ASSERT(l_geo_id, "AffineBodyPrismaticJointLimit requires `l_geo_id` attribute on edges");
                auto l_geo_id_view = l_geo_id->view();
                auto r_geo_id      = sc->edges().find<IndexT>("r_geo_id");
                UIPC_ASSERT(r_geo_id, "AffineBodyPrismaticJointLimit requires `r_geo_id` attribute on edges");
                auto r_geo_id_view = r_geo_id->view();
                auto l_inst_id     = sc->edges().find<IndexT>("l_inst_id");
                UIPC_ASSERT(l_inst_id, "AffineBodyPrismaticJointLimit requires `l_inst_id` attribute on edges");
                auto l_inst_id_view = l_inst_id->view();
                auto r_inst_id      = sc->edges().find<IndexT>("r_inst_id");
                UIPC_ASSERT(r_inst_id, "AffineBodyPrismaticJointLimit requires `r_inst_id` attribute on edges");
                auto r_inst_id_view = r_inst_id->view();

                auto lower_attr = sc->edges().find<Float>("limit/lower");
                UIPC_ASSERT(lower_attr, "AffineBodyPrismaticJointLimit requires `limit/lower` attribute on edges");
                auto lower_view = lower_attr->view();

                auto upper_attr = sc->edges().find<Float>("limit/upper");
                UIPC_ASSERT(upper_attr, "AffineBodyPrismaticJointLimit requires `limit/upper` attribute on edges");
                auto upper_view = upper_attr->view();

                auto strength_attr = sc->edges().find<Float>("limit/strength");
                UIPC_ASSERT(strength_attr, "AffineBodyPrismaticJointLimit requires `limit/strength` attribute on edges");
                auto strength_view = strength_attr->view();

                auto edges = sc->edges().topo().view();

                for(auto&& [i, e] : enumerate(edges))
                {
                    IndexT l_gid = l_geo_id_view[i];
                    IndexT r_gid = r_geo_id_view[i];
                    IndexT l_iid = l_inst_id_view[i];
                    IndexT r_iid = r_inst_id_view[i];

                    auto* left_sc  = info.body_geo(geo_slots, l_gid);
                    auto* right_sc = info.body_geo(geo_slots, r_gid);

                    UIPC_ASSERT(l_iid >= 0
                                    && l_iid < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyPrismaticJointLimit: left instance ID {} out of range [0, {})",
                                l_iid,
                                left_sc->instances().size());
                    UIPC_ASSERT(r_iid >= 0
                                    && r_iid < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyPrismaticJointLimit: right instance ID {} out of range [0, {})",
                                r_iid,
                                right_sc->instances().size());

                    Vector24 ref;
                    ref.segment<12>(0) =
                        transform_to_q(left_sc->transforms().view()[l_iid]);
                    ref.segment<12>(12) =
                        transform_to_q(right_sc->transforms().view()[r_iid]);
                    h_ref_qs.push_back(ref);

                    UIPC_ASSERT(lower_view[i] <= upper_view[i],
                                "AffineBodyPrismaticJointLimit: requires `limit/lower <= limit/upper` on edge {}, but got lower={} upper={}",
                                i,
                                lower_view[i],
                                upper_view[i]);
                    h_lowers.push_back(lower_view[i]);
                    h_uppers.push_back(upper_view[i]);
                    h_strengths.push_back(strength_view[i]);
                }
            });

        // Sanity check: limit constitution reuses body_ids/rest_cs/rest_ts/init_distances
        // from AffineBodyPrismaticJoint via the shared index `I`. Both must
        // therefore enumerate exactly the same (geo, edge) sequence.
        UIPC_ASSERT(h_ref_qs.size() == prismatic_joint->h_body_ids.size(),
                    "AffineBodyPrismaticJointLimit: joint count {} must equal "
                    "AffineBodyPrismaticJoint joint count {} (index alignment required)",
                    h_ref_qs.size(),
                    prismatic_joint->h_body_ids.size());

        ref_qs.copy_from(h_ref_qs);
        lowers.copy_from(h_lowers);
        uppers.copy_from(h_uppers);
        strengths.copy_from(h_strengths);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(ref_qs.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(ref_qs.size(),
                   [body_ids = prismatic_joint->body_ids.cviewer().name("body_ids"),
                    rest_cs = prismatic_joint->rest_cs.cviewer().name("rest_cs"),
                    rest_ts = prismatic_joint->rest_ts.cviewer().name("rest_ts"),
                    init_distances = prismatic_joint->init_distances.cviewer().name("init_distances"),
                    ref_qs    = ref_qs.cviewer().name("ref_qs"),
                    lowers    = lowers.cviewer().name("lowers"),
                    uppers    = uppers.cviewer().name("uppers"),
                    strengths = strengths.cviewer().name("strengths"),
                    qs        = info.qs().cviewer().name("qs"),
                    q_prevs   = info.q_prevs().cviewer().name("q_prevs"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bid = body_ids(I);

                       const Vector6& C_bar = rest_cs(I);
                       const Vector6& t_bar = rest_ts(I);
                       Vector12       qk    = qs(bid[0]);
                       Vector12       ql    = qs(bid[1]);
                       Float          x;
                       compute_absolute_distance(x, C_bar, t_bar, qk, ql);

                       Float init_d   = init_distances(I);
                       Float lower    = lowers(I) - init_d;
                       Float upper    = uppers(I) - init_d;
                       Float strength = strengths(I);

                       Float E = joint_limit::eval_penalty_energy<Float>(x, lower, upper, strength);

                       Es(I) = E;
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(2 * ref_qs.size());
        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * ref_qs.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        auto gradient_only = info.gradient_only();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(ref_qs.size(),
                   [body_ids = prismatic_joint->body_ids.cviewer().name("body_ids"),
                    rest_cs = prismatic_joint->rest_cs.cviewer().name("rest_cs"),
                    rest_ts = prismatic_joint->rest_ts.cviewer().name("rest_ts"),
                    init_distances = prismatic_joint->init_distances.cviewer().name("init_distances"),
                    ref_qs    = ref_qs.cviewer().name("ref_qs"),
                    lowers    = lowers.cviewer().name("lowers"),
                    uppers    = uppers.cviewer().name("uppers"),
                    strengths = strengths.cviewer().name("strengths"),
                    qs        = info.qs().cviewer().name("qs"),
                    q_prevs   = info.q_prevs().cviewer().name("q_prevs"),
                    G12s      = info.gradients().viewer().name("G12s"),
                    H12x12s   = info.hessians().viewer().name("H12x12s"),
                    gradient_only] __device__(int I) mutable
                   {
                       Vector2i bid = body_ids(I);

                       const Vector6& C_bar = rest_cs(I);
                       const Vector6& t_bar = rest_ts(I);

                       Vector12 qk = qs(bid[0]);
                       Vector12 ql = qs(bid[1]);
                       Float    x;
                       compute_absolute_distance(x, C_bar, t_bar, qk, ql);

                       Float init_d   = init_distances(I);
                       Float lower    = lowers(I) - init_d;
                       Float upper    = uppers(I) - init_d;
                       Float strength = strengths(I);

                       Float dE_dx   = 0.0f;
                       Float d2E_dx2 = 0.0f;
                       joint_limit::eval_penalty_derivatives<Float>(
                           x, lower, upper, strength, dE_dx, d2E_dx2);

                       Vector24 dx_dq;
                       compute_absolute_distance_derivative(dx_dq, C_bar, t_bar, qk, ql);

                       Vector24               G = dE_dx * dx_dq;
                       DoubletVectorAssembler DVA{G12s};
                       DVA.segment<2>(2 * I).write(bid, G);

                       if(gradient_only)
                           return;

                       Matrix24x24 H = d2E_dx2 * (dx_dq * dx_dq.transpose());

                       if(dE_dx != 0.0f)
                       {
                           compute_absolute_distance_hessian(H, dE_dx, C_bar, t_bar, qk, ql);
                       }

                       TripletMatrixAssembler TMA{H12x12s};
                       TMA.half_block<2>(HalfHessianSize * I).write(bid, H);
                   });
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointLimit);

}  // namespace uipc::backend::cuda
