#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>
#include <time_integrator/time_integrator.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/offset_count_collection.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/enumerate.h>

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

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;

    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        list<Vector2i> body_ids_list;
        list<Vector6>  rest_c_list;
        list<Vector6>  rest_t_list;
        list<Vector6>  rest_n_list;
        list<Vector6>  rest_b_list;
        list<Float>    strength_ratio_list;

        info.for_each(
            geo_slots,
            [&](geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                U64  uid_value = uid->view()[0];
                UIPC_ASSERT(uid_value == ConstitutionUID,
                            "AffineBodyPrismaticJoint: Geometry constitution UID mismatch");

                auto sc = geo.as<geometry::SimplicialComplex>();

                auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "AffineBodyPrismaticJoint: Geometry must have 'geo_ids' attribute on `edges`");
                auto geo_ids_view = geo_ids->view();

                auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
                UIPC_ASSERT(inst_ids, "AffineBodyPrismaticJoint: Geometry must have 'inst_ids' attribute on `edges`");
                auto inst_ids_view = inst_ids->view();

                auto strength_ratio = sc->edges().find<Float>("strength_ratio");
                UIPC_ASSERT(strength_ratio, "AffineBodyPrismaticJoint: Geometry must have 'strength_ratio' attribute on `edges`");
                auto strength_ratio_view = strength_ratio->view();

                auto Normal = [&](const Vector3& W) -> Vector3
                {
                    Vector3 ref = abs(W.dot(Vector3(1, 0, 0))) < 0.9 ?
                                      Vector3(1, 0, 0) :
                                      Vector3(0, 1, 0);

                    Vector3 U = ref.cross(W).normalized();
                    Vector3 V = W.cross(U).normalized();

                    return V;
                };

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();
                for(auto&& [i, e] : enumerate(Es))
                {
                    Vector3 P0 = Ps[e[0]];
                    Vector3 P1 = Ps[e[1]];
                    UIPC_ASSERT((P0 - P1).squaredNorm() > 0,
                                "AffineBodyPrismaticJoint: Edge positions must not be too close");

                    Vector2i geo_id  = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                         info.body_id(geo_id(1), inst_id(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                    auto right_sc = info.body_geo(geo_slots, geo_id(1));

                    UIPC_ASSERT(inst_id(0) >= 0
                                    && inst_id(0) < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Left instance ID {} is out of range [0, {})",
                                inst_id(0),
                                left_sc->instances().size());
                    UIPC_ASSERT(inst_id(1) >= 0
                                    && inst_id(1) < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Right instance ID {} is out of range [0, {})",
                                inst_id(1),
                                right_sc->instances().size());

                    Transform LT{left_sc->transforms().view()[inst_id(0)]};
                    Transform RT{right_sc->transforms().view()[inst_id(1)]};

                    Vector3 tangent   = (P1 - P0).normalized();
                    Vector3 normal    = Normal(tangent);
                    Vector3 bitangent = normal.cross(tangent).normalized();

                    Vector6 rest_position_c;
                    rest_position_c.segment<3>(0) = LT.inverse() * P0;  // ci_bar
                    rest_position_c.segment<3>(3) = RT.inverse() * P0;  // cj_bar
                    rest_c_list.push_back(rest_position_c);

                    Matrix3x3 LT_rotation = LT.rotation();
                    Matrix3x3 RT_rotation = RT.rotation();

                    Vector6 rest_vec_t;
                    rest_vec_t.segment<3>(0) = LT_rotation.inverse() * tangent;  // ti_bar
                    rest_vec_t.segment<3>(3) = RT_rotation.inverse() * tangent;  // tj_bar
                    rest_t_list.push_back(rest_vec_t);

                    Vector6 rest_vec_n;
                    rest_vec_n.segment<3>(0) = LT_rotation.inverse() * normal;  // ni_bar
                    rest_vec_n.segment<3>(3) = RT_rotation.inverse() * normal;  // nj_bar
                    rest_n_list.push_back(rest_vec_n);

                    Vector6 rest_vec_b;
                    rest_vec_b.segment<3>(0) = LT_rotation.inverse() * bitangent;  // bi_bar
                    rest_vec_b.segment<3>(3) = RT_rotation.inverse() * bitangent;  // bj_bar
                    rest_b_list.push_back(rest_vec_b);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
            });

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

        body_ids.copy_from(h_body_ids);
        rest_cs.copy_from(h_rest_cs);
        rest_ts.copy_from(h_rest_ts);
        rest_ns.copy_from(h_rest_ns);
        rest_bs.copy_from(h_rest_bs);
        strength_ratios.copy_from(h_strength_ratios);
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

                       if(!gradient_only)
                       {
                           // Get Hessian based on Frame F01
                           Matrix9x9 H01;
                           PJ::ddE01ddF01(H01, kappa, F01);

                           // Ensure H01 is SPD
                           make_spd(H01);

                           Matrix24x24 J01T_H01_J01;
                           PJ::J01T_H01_J01<Float>(J01T_H01_J01,
                                                   H01,
                                                   rest_c.segment<3>(0),  // ci_bar
                                                   rest_t.segment<3>(0),  // ti_bar
                                                   rest_c.segment<3>(3),  // cj_bar
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
                       }
                   });
    };

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJoint);

class AffineBodyDrivingPrismaticJoint : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    static constexpr SizeT StencilSize     = 2;

    static constexpr U64 ConstraintUID = 21;

    SimSystemSlot<AffineBodyPrismaticJoint> prismatic_joint;


    // Host
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;

    vector<Vector2i> h_body_ids;

    vector<Vector6> h_rest_position;
    vector<Vector6> h_rest_tangent;

    vector<IndexT> h_is_constrained;
    vector<Float>  h_strength_ratios;
    vector<IndexT> h_is_passive;

    vector<Float> h_init_distances;
    vector<Float> h_aim_distances;
    vector<Float> h_current_distances;

    bool is_constrained_changed  = false;
    bool strength_ratios_changed = false;
    bool is_passive_changed      = false;
    bool aim_distances_changed   = false;


    // Device
    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6>  rest_positions;
    muda::DeviceBuffer<Vector6>  rest_tangents;
    muda::DeviceBuffer<IndexT>   is_constrained;
    muda::DeviceBuffer<Float>    strength_ratios;
    muda::DeviceBuffer<IndexT>   is_passive;
    muda::DeviceBuffer<Float>    init_distances;
    muda::DeviceBuffer<Float>    aim_distances;
    muda::DeviceBuffer<Float>    current_distances;

    void do_build(BuildInfo& info) override
    {

        prismatic_joint = require<AffineBodyPrismaticJoint>();

        on_write_scene([this]() { write_scene(); });
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();
        h_geo_joint_offsets_counts.resize(info.anim_inter_geo_infos().size());

        list<Vector2i> body_ids_list;
        list<Vector6>  rest_position_list;
        list<Vector6>  rest_tangent_list;
        list<IndexT>   is_constrained_list;
        list<Float>    strength_ratios_list;
        list<IndexT>   is_passive_list;
        list<Float>    init_distances_list;
        list<Float>    aim_distances_list;
        list<Float>    current_distances_list;

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
                // check consitudtion uid
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

                // get geo_ids and inst_ids
                auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "AffineBodyDrivingPrismaticJoint geometry must have 'geo_ids' attribute on `edges`");
                auto geo_ids_view = geo_ids->view();

                auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
                UIPC_ASSERT(inst_ids, "AffineBodyDrivingPrismaticJoint: Geometry must have 'inst_ids' attribute on `edges`");
                auto inst_ids_view = inst_ids->view();

                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();
                std::ranges::copy(is_constrained_view,
                                  std::back_inserter(is_constrained_list));

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                auto strength_ratios_view = strength_ratios->view();
                std::ranges::copy(strength_ratios_view,
                                  std::back_inserter(strength_ratios_list));

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingPrismaticJoint: Geometry must have 'is_passive' attribute on `edges`")
                auto is_passive_view = is_passive->view();
                std::ranges::copy(is_passive_view, std::back_inserter(is_passive_list));

                auto init_distances = sc->edges().find<Float>("init_distance");
                UIPC_ASSERT(init_distances, "AffineBodyDrivingPrismaticJoint: Geometry must have 'init_distance' attribute on `edges`")
                auto init_ditances_view = init_distances->view();
                std::ranges::copy(init_ditances_view, std::back_inserter(init_distances_list));

                auto aim_distances = sc->edges().find<Float>("aim_distance");
                UIPC_ASSERT(aim_distances, "AffineBodyDrivingPrismaticJoint: Geometry must have 'aim_distance' attribute on `edges`")
                auto aim_distances_view = aim_distances->view();
                std::ranges::copy(aim_distances_view, std::back_inserter(aim_distances_list));

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();

                for(auto&& [i, e] : enumerate(Es))
                {
                    Vector2i geo_id  = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    Vector3 P0 = Ps[e[0]];
                    Vector3 P1 = Ps[e[1]];

                    UIPC_ASSERT((P0 - P1).squaredNorm() > 0,
                                R"(AffineBodyDrivingPrismaticJoint: Edge with zero length detected,
Joint GeometryID = {},
LinkGeoIDs       = ({}, {}),
LinkInstIDs      = ({}, {}),
Edge             = ({}, {}))",
                                I.geo_info().geo_id,
                                geo_id(0),
                                geo_id(1),
                                inst_id(0),
                                inst_id(1),
                                e(0),
                                e(1));

                    Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                         info.body_id(geo_id(1), inst_id(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                    auto right_sc = info.body_geo(geo_slots, geo_id(1));

                    UIPC_ASSERT(inst_id(0) >= 0
                                    && inst_id(0) < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyDrivingPrismaticJoint: Left instance ID {} is out of range [0, {})",
                                inst_id(0),
                                left_sc->instances().size());
                    UIPC_ASSERT(inst_id(1) >= 0
                                    && inst_id(1) < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyDrivingPrismaticJoint: Right instance ID {} is out of range [0, {})",
                                inst_id(1),
                                right_sc->instances().size());

                    Vector3   tangent = (P1 - P0).normalized();
                    Transform LT{left_sc->transforms().view()[inst_id(0)]};
                    Transform RT{right_sc->transforms().view()[inst_id(1)]};

                    // position
                    Vector6 rest_position;
                    rest_position.segment<3>(0) = LT.inverse() * P0;
                    rest_position.segment<3>(3) = RT.inverse() * P0;
                    rest_position_list.push_back(rest_position);

                    // tangent
                    Vector6 rest_tangent;
                    rest_tangent.segment<3>(0) = LT.rotation().inverse() * tangent;
                    rest_tangent.segment<3>(3) = RT.rotation().inverse() * tangent;
                    rest_tangent_list.push_back(rest_tangent);
                }
                joint_offset++;
            });
        // scan to compute offsets
        h_geo_joint_offsets_counts.scan();

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_is_constrained.resize(is_constrained_list.size());
        std::ranges::move(is_constrained_list, h_is_constrained.begin());

        h_is_passive.resize(is_passive_list.size());
        std::ranges::move(is_passive_list, h_is_passive.begin());

        h_strength_ratios.resize(strength_ratios_list.size());
        std::ranges::move(strength_ratios_list, h_strength_ratios.begin());

        h_rest_position.resize(rest_position_list.size());
        std::ranges::move(rest_position_list, h_rest_position.begin());

        h_rest_tangent.resize(rest_tangent_list.size());
        std::ranges::move(rest_tangent_list, h_rest_tangent.begin());

        h_init_distances.resize(init_distances_list.size());
        std::ranges::move(init_distances_list, h_init_distances.begin());

        h_aim_distances.resize(aim_distances_list.size());
        std::ranges::move(aim_distances_list, h_aim_distances.begin());

        body_ids.copy_from(h_body_ids);
        is_constrained.copy_from(h_is_constrained);
        is_passive.copy_from(h_is_passive);
        strength_ratios.copy_from(h_strength_ratios);

        rest_positions.copy_from(h_rest_position);
        rest_tangents.copy_from(h_rest_tangent);
        init_distances.copy_from(h_init_distances);
        aim_distances.copy_from(h_aim_distances);
        current_distances = init_distances;
    }


    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto  geo_slots       = world().scene().geometries();
        SizeT geo_joint_index = 0;

        is_constrained_changed  = false;
        strength_ratios_changed = false;
        is_passive_changed      = false;
        aim_distances_changed   = false;

        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyDrivingPrismaticJoint: Geometry must be a simplicial complex");

                auto joint_geo_id = I.geo_info().geo_id;
                auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];

                UIPC_ASSERT(sc->edges().size() == count,
                            "AffineBodyDrivingPrismaticJoint: Geometry edges size {} mismatch with joint count {}",
                            sc->edges().size(),
                            count);

                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                {
                    auto is_constrained_view = is_constrained->view();
                    auto dst = span{h_is_constrained}.subspan(offset, count);
                    std::ranges::copy(is_constrained_view, dst.begin());
                    is_constrained_changed = true;
                }

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingPrismaticJoint: Geometry must have 'is_passive' attribute on `edges`")
                {
                    auto is_passive_view = is_passive->view();
                    auto dst = span{h_is_passive}.subspan(offset, count);
                    std::ranges::copy(is_passive_view, dst.begin());
                    is_passive_changed = true;
                }

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingPrismaticJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                {
                    auto strength_ratios_view = strength_ratios->view();
                    auto dst = span{h_strength_ratios}.subspan(offset, count);
                    std::ranges::copy(strength_ratios_view, dst.begin());
                    strength_ratios_changed = true;
                }

                auto aim_distances = sc->edges().find<Float>("aim_distance");
                UIPC_ASSERT(aim_distances, "AffineBodyDrivingPrismaticJoint: Geometry must have 'aim_distances' attribute on `edges`");
                {
                    auto aim_angles_view = aim_distances->view();
                    auto dst = span{h_aim_distances}.subspan(offset, count);
                    std::ranges::copy(aim_angles_view, dst.begin());
                    aim_distances_changed = true;
                }
                ++geo_joint_index;
            });

        if(aim_distances_changed)
            aim_distances.copy_from(h_aim_distances);
        if(is_constrained_changed)
            is_constrained.copy_from(h_is_constrained);
        if(strength_ratios_changed)
            strength_ratios.copy_from(h_strength_ratios);
        if(is_passive_changed)
            is_passive.copy_from(h_is_passive);
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
                UIPC_ASSERT(sc, "AffineBodyDrivingPrismaticJoint: Geometry must be a simplicial complex");

                auto distance = sc->edges().find<Float>("distance");

                if(distance)
                {
                    auto distance_view = view(*distance);
                    auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];
                    UIPC_ASSERT(distance_view.size() == count,
                                "AffineBodyDrivingPrismaticJoint: distance attribute size {} mismatch with joint count {}",
                                distance_view.size(),
                                count);

                    auto dst = span{h_current_distances}.subspan(offset, count);
                    std::ranges::copy(dst, distance_view.begin());
                }

                ++geo_joint_index;
            });
    }

    void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(body_ids.size());
        info.gradient_count(2 * body_ids.size());
        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * body_ids.size());
    }


    void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace DPJ = sym::affine_body_driving_prismatic_joint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    is_constrained = is_constrained.cviewer().name("is_constrained"),
                    is_passive = is_passive.cviewer().name("is_passive"),
                    strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                    rest_positions = rest_positions.cviewer().name("rest_positions"),
                    rest_tangents = rest_tangents.cviewer().name("rest_tangents"),
                    init_distances = init_distances.cviewer().name("init_distances"),
                    curr_distances = current_distances.cviewer().name("current_distances"),
                    aim_distances = aim_distances.cviewer().name("aim_distances"),
                    qs = info.qs().cviewer().name("qs"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    is_fixed = info.is_fixed().cviewer().name("is_fixed"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bids        = body_ids(I);
                       auto     constrained = is_constrained(I);
                       if(constrained == 0)
                       {
                           Es(I) = 0.0;
                           return;
                       }
                       Float kappa = strength_ratios(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       auto passive      = is_passive(I);
                       auto aim_distance = aim_distances(I);
                       if(passive == 1)
                       {
                           // resist external forces passively
                           aim_distance = curr_distances(I);
                       }

                       Float d_tidle = aim_distance + init_distances(I);

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

                       // E0 = 1/2 * kappa * (T_i \cdot (C_j - C_i) - d_tidle)^2
                       // E1 = 1/2 * kappa * (T_j \cdot (C_i - C_j) - d_tidle)^2

                       //    Float E01;
                       //    DPJ::E01<Float>(E01, kappa, F01, d_tidle);
                       //    Es(I) = E01;
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
                body_ids.size(),
                [body_ids = body_ids.cviewer().name("body_ids"),
                 is_constrained = is_constrained.cviewer().name("is_constrained"),
                 is_passive = is_passive.cviewer().name("is_passive"),
                 strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                 rest_positions = rest_positions.cviewer().name("rest_positions"),
                 rest_tangents = rest_tangents.cviewer().name("rest_tangents"),
                 init_distances = init_distances.cviewer().name("init_distances"),
                 curr_distances = current_distances.cviewer().name("current_distances"),
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

                    Float d_tidle = aim_distance + init_distances(I);

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


    BufferDump curr_distances_dump;

    bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_distances_dump.dump(fmt::format("{}current_angle.{}", path, frame),
                                        current_distances);
    }

    bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_distances_dump.load(fmt::format("{}current_distances.{}", path, frame));
    }

    void do_apply_recover(RecoverInfo& info) override
    {
        curr_distances_dump.apply_to(current_distances);
    }

    void do_clear_recover(RecoverInfo& info) override
    {
        curr_distances_dump.clean_up();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingPrismaticJoint);

class AffineBodyDrivingPrismaticJointTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyDrivingPrismaticJoint> driving_prismatic_joint;
    SimSystemSlot<AffineBodyDynamics>              affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        driving_prismatic_joint = require<AffineBodyDrivingPrismaticJoint>();
        affine_body_dynamics    = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override
    {
        // do noting here
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace DPJ = sym::affine_body_driving_prismatic_joint;
        auto& dpj     = driving_prismatic_joint;


        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(dpj->body_ids.size(),
                   [body_ids = dpj->body_ids.cviewer().name("body_ids"),
                    rest_positions = dpj->rest_positions.cviewer().name("rest_position"),
                    rest_tangents = dpj->rest_tangents.cviewer().name("rest_tangents"),
                    current_distances = dpj->current_distances.viewer().name("current_distances"),
                    init_distances = dpj->init_distances.cviewer().name("init_distances"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

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

                       Float distance;
                       DPJ::Distance<Float>(distance, F01_q);

                       current_distances(I) = distance - init_distances(I);
                   });
    }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingPrismaticJointTimeIntegrator);


}  // namespace uipc::backend::cuda