#include <numbers>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <time_integrator/time_integrator.h>
#include <affine_body/inter_affine_body_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <uipc/common/enumerate.h>
namespace uipc::backend::cuda
{
class AffineBodyRevoluteJoint final : public InterAffineBodyConstitution
{
  public:
    using InterAffineBodyConstitution::InterAffineBodyConstitution;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    static constexpr U64 ConstitutionUID = 18;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    vector<Vector2i> h_body_ids;
    // [    body0   |   body1    ]
    // [    x0, x1  |   x2, x3   ]
    vector<Vector12> h_rest_positions;
    vector<Float>    h_strength_ratio;

    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector12> rest_positions;
    muda::DeviceBuffer<Float>    strength_ratio;


    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        list<Vector2i> body_ids_list;
        list<Vector12> rest_positions_list;
        list<Float>    strength_ratio_list;

        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                U64  uid_value = uid->view()[0];
                UIPC_ASSERT(uid_value == ConstitutionUID,
                            "AffineBodyRevoluteJoint: Geometry constitution UID mismatch");

                auto joint_geo_id = I.geo_info().geo_id;

                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyRevoluteJoint: Geometry must be a simplicial complex");

                auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "AffineBodyRevoluteJoint: Geometry must have 'geo_ids' attribute on `edges`");
                auto geo_ids_view = geo_ids->view();

                auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
                UIPC_ASSERT(inst_ids, "AffineBodyRevoluteJoint: Geometry must have 'inst_ids' attribute on `edges`");
                auto inst_ids_view = inst_ids->view();

                auto strength_ratio = sc->edges().find<Float>("strength_ratio");
                UIPC_ASSERT(strength_ratio, "AffineBodyRevoluteJoint: Geometry must have 'strength_ratio' attribute on `edges`");
                auto strength_ratio_view = strength_ratio->view();

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();
                for(auto&& [i, e] : enumerate(Es))
                {
                    Vector2i geo_id  = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    Vector3 P0  = Ps[e[0]];
                    Vector3 P1  = Ps[e[1]];
                    Vector3 mid = (P0 + P1) / 2;
                    Vector3 Dir = (P1 - P0);

                    UIPC_ASSERT(Dir.norm() > 1e-12,
                                R"(AffineBodyRevoluteJoint: Edge with zero length detected,
Joint GeometryID = {},
LinkGeoIDs       = ({}, {}),
LinkInstIDs      = ({}, {}),
Edge             = ({}, {}))",
                                joint_geo_id,
                                geo_id(0),
                                geo_id(1),
                                inst_id(0),
                                inst_id(1),
                                e(0),
                                e(1));

                    Vector3 HalfAxis = Dir.normalized() / 2;

                    // Re-define P0 and P1 to be symmetric around the mid-point
                    P0 = mid - HalfAxis;
                    P1 = mid + HalfAxis;

                    Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                         info.body_id(geo_id(1), inst_id(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                    auto right_sc = info.body_geo(geo_slots, geo_id(1));

                    UIPC_ASSERT(inst_id(0) >= 0
                                    && inst_id(0) < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyRevoluteJoint: Left instance ID {} is out of range [0, {})",
                                inst_id(0),
                                left_sc->instances().size());
                    UIPC_ASSERT(inst_id(1) >= 0
                                    && inst_id(1) < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyRevoluteJoint: Right instance ID {} is out of range [0, {})",
                                inst_id(1),
                                right_sc->instances().size());

                    Transform LT{left_sc->transforms().view()[inst_id(0)]};
                    Transform RT{right_sc->transforms().view()[inst_id(1)]};

                    Vector12 rest_pos;
                    rest_pos.segment<3>(0) = LT.inverse() * P0;  // x0_bar
                    rest_pos.segment<3>(3) = LT.inverse() * P1;  // x1_bar

                    rest_pos.segment<3>(6) = RT.inverse() * P0;  // x2_bar
                    rest_pos.segment<3>(9) = RT.inverse() * P1;  // x3_bar
                    rest_positions_list.push_back(rest_pos);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
            });

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_rest_positions.resize(rest_positions_list.size());
        std::ranges::move(rest_positions_list, h_rest_positions.begin());

        h_strength_ratio.resize(strength_ratio_list.size());
        std::ranges::move(strength_ratio_list, h_strength_ratio.begin());

        body_ids.copy_from(h_body_ids);
        rest_positions.copy_from(h_rest_positions);
        strength_ratio.copy_from(h_strength_ratio);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());  // one energy per joint
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace RJ = sym::affine_body_revolute_joint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_positions = rest_positions.cviewer().name("rest_positions"),
                    strength_ratio = strength_ratio.cviewer().name("strength_ratio"),
                    body_masses = info.body_masses().viewer().name("body_masses"),
                    qs = info.qs().viewer().name("qs"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Float kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       const Vector12& X_bar = rest_positions(I);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       ABDJacobi Js[4] = {ABDJacobi{X_bar.segment<3>(0)},
                                          ABDJacobi{X_bar.segment<3>(3)},
                                          ABDJacobi{X_bar.segment<3>(6)},
                                          ABDJacobi{X_bar.segment<3>(9)}};

                       Vector12 X;
                       X.segment<3>(0) = Js[0].point_x(q_i);
                       X.segment<3>(3) = Js[1].point_x(q_i);

                       X.segment<3>(6) = Js[2].point_x(q_j);
                       X.segment<3>(9) = Js[3].point_x(q_j);


                       Float E0 = (X.segment<3>(0) - X.segment<3>(6)).squaredNorm();
                       Float E1 = (X.segment<3>(3) - X.segment<3>(9)).squaredNorm();

                       // energy = 1/2 * kappa * (||x0 - x2||^2 + ||x1 - x3||^2)

                       Es(I) = kappa / 2 * (E0 + E1);
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
        namespace RJ       = sym::affine_body_revolute_joint;
        auto gradient_only = info.gradient_only();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                body_ids.size(),
                [body_ids = body_ids.cviewer().name("body_ids"),
                 rest_positions = rest_positions.cviewer().name("rest_positions"),
                 strength_ratio = strength_ratio.cviewer().name("strength_ratio"),
                 body_masses = info.body_masses().viewer().name("body_masses"),
                 qs          = info.qs().viewer().name("qs"),
                 G12s        = info.gradients().viewer().name("G12s"),
                 H12x12s     = info.hessians().viewer().name("H12x12s"),
                 gradient_only] __device__(int I)
                {
                    Vector2i        bids  = body_ids(I);
                    const Vector12& X_bar = rest_positions(I);

                    Vector12 q_i = qs(bids(0));
                    Vector12 q_j = qs(bids(1));

                    ABDJacobi Js[4] = {ABDJacobi{X_bar.segment<3>(0)},
                                       ABDJacobi{X_bar.segment<3>(3)},
                                       ABDJacobi{X_bar.segment<3>(6)},
                                       ABDJacobi{X_bar.segment<3>(9)}};

                    Vector12 X;
                    X.segment<3>(0) = Js[0].point_x(q_i);
                    X.segment<3>(3) = Js[1].point_x(q_i);

                    X.segment<3>(6) = Js[2].point_x(q_j);
                    X.segment<3>(9) = Js[3].point_x(q_j);

                    Vector3 D02 = Js[0].point_x(q_i) - Js[2].point_x(q_j);
                    Vector3 D13 = Js[1].point_x(q_i) - Js[3].point_x(q_j);
                    Float   K =
                        strength_ratio(I)
                        * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

                    // Fill Body Gradient:
                    {
                        // G = 0.5 * kappa * (J0^T * (x0 - x2) + J1^T * (x1 - x3))
                        Vector12 G_i = K * (Js[0].T() * D02 + Js[1].T() * D13);
                        G12s(2 * I + 0).write(bids(0), G_i);
                    }
                    {
                        // G = 0.5 * kappa * (J2^T * (x2 - x0) + J3^T * (x3 - x1))
                        Vector12 G_j = K * (Js[2].T() * (-D02) + Js[3].T() * (-D13));
                        G12s(2 * I + 1).write(bids(1), G_j);
                    }

                    // Fill Body Hessian:
                    if(!gradient_only)
                    {
                        {
                            Matrix12x12 H_ii;
                            RJ::Hess(H_ii,
                                     K,
                                     Js[0].x_bar(),
                                     Js[0].x_bar(),
                                     Js[1].x_bar(),
                                     Js[1].x_bar());
                            H12x12s(HalfHessianSize * I + 0).write(bids(0), bids(0), H_ii);
                        }
                        {
                            Matrix12x12 H;
                            Vector2i    lr = bids;
                            RJ::Hess(H,
                                     -K,
                                     Js[0].x_bar(),
                                     Js[2].x_bar(),
                                     Js[1].x_bar(),
                                     Js[3].x_bar());
                            if(bids(0) > bids(1))
                            {
                                H.transposeInPlace();
                                lr = Vector2i{bids(1), bids(0)};
                            }
                            H12x12s(HalfHessianSize * I + 1).write(lr(0), lr(1), H);
                        }
                        {
                            Matrix12x12 H_jj;
                            RJ::Hess(H_jj,
                                     K,
                                     Js[2].x_bar(),
                                     Js[2].x_bar(),
                                     Js[3].x_bar(),
                                     Js[3].x_bar());
                            H12x12s(HalfHessianSize * I + 2).write(bids(1), bids(1), H_jj);
                        }
                    }
                });
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};

REGISTER_SIM_SYSTEM(AffineBodyRevoluteJoint);

class AffineBodyDrivingRevoluteJoint : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;

    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    static constexpr SizeT StencilSize     = 2;

    static constexpr U64 ConstraintUID = 19;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;

    // Host
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;

    vector<Vector2i> h_body_ids;

    vector<Vector6> h_rest_axis;
    vector<Vector6> h_rest_normals;

    vector<IndexT> h_is_constrained;
    vector<Float>  h_strength_ratios;
    vector<IndexT> h_is_passive;

    vector<Float> h_init_angles;
    vector<Float> h_aim_angles;
    vector<Float> h_current_angles;

    bool is_constrained_changed  = false;
    bool strength_ratios_changed = false;
    bool is_passive_changed      = false;
    bool aim_angles_changed      = false;

    // Device
    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6>  rest_axis;
    muda::DeviceBuffer<Vector6>  rest_normals;
    muda::DeviceBuffer<IndexT>   is_constrained;
    muda::DeviceBuffer<Float>    strength_ratios;
    muda::DeviceBuffer<IndexT>   is_passive;
    muda::DeviceBuffer<Float>    init_angles;
    muda::DeviceBuffer<Float>    aim_angles;
    muda::DeviceBuffer<Float>    current_angles;

    void do_build(BuildInfo& info) override
    {

        revolute_joint = require<AffineBodyRevoluteJoint>();

        on_write_scene([this]() { write_scene(); });
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {

        auto geo_slots = world().scene().geometries();

        h_geo_joint_offsets_counts.resize(info.anim_inter_geo_infos().size());

        list<Vector2i> body_ids_list;
        list<Vector6>  rest_axis_list;
        list<Vector6>  rest_normals_list;
        list<IndexT>   is_constrained_list;
        list<Float>    strength_ratios_list;
        list<IndexT>   is_passive_list;
        list<Float>    init_angles_list;
        list<Float>    aim_angles_list;

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
                    UIPC_ASSERT(constitution_uid, "AffineBodyDrivingRevoluteJoint: Geometry must have 'constitution_uid' attribute");
                    U64 uid_value = constitution_uid->view()[0];
                    UIPC_ASSERT(uid_value == AffineBodyRevoluteJoint::ConstitutionUID,
                                "AffineBodyDrivingRevoluteJoint: Geometry constitution UID mismatch");
                }

                // get simplicial complex
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyDrivingRevoluteJoint geometry must be SimplicialComplex");

                h_geo_joint_offsets_counts.counts()[joint_offset] = sc->edges().size();
                UIPC_ASSERT(h_geo_joint_offsets_counts.counts()[joint_offset] > 0,
                            "AffineBodyDrivingRevoluteJoint: Geometry must have at least one edge");

                // get geo_ids and inst_ids
                auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "AffineBodyDrivingRevoluteJoint geometry must have 'geo_ids' attribute on `edges`");
                auto geo_ids_view = geo_ids->view();

                auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
                UIPC_ASSERT(inst_ids, "AffineBodyDrivingRevoluteJoint: Geometry must have 'inst_ids' attribute on `edges`");
                auto inst_ids_view = inst_ids->view();

                auto is_constrained = sc->edges().find<IndexT>(builtin::is_constrained);
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();
                std::ranges::copy(is_constrained_view,
                                  std::back_inserter(is_constrained_list));

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                auto strength_ratios_view = strength_ratios->view();
                std::ranges::copy(strength_ratios_view,
                                  std::back_inserter(strength_ratios_list));

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_passive' attribute on `edges`")
                auto is_passive_view = is_passive->view();
                std::ranges::copy(is_passive_view, std::back_inserter(is_passive_list));

                auto init_angles = sc->edges().find<Float>("init_angle");
                UIPC_ASSERT(init_angles, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_constrained' attribute on `edges`");
                auto init_angles_view = init_angles->view();
                std::ranges::copy(init_angles_view, std::back_inserter(init_angles_list));

                auto aim_angles = sc->edges().find<Float>("aim_angle");
                UIPC_ASSERT(aim_angles, "AffineBodyDrivingRevoluteJoint: Geometry must have 'aim_angles' attribute on `edges`");
                auto aim_angles_view = aim_angles->view();
                std::ranges::copy(aim_angles_view, std::back_inserter(aim_angles_list));

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();

                auto toNormal = [&](const Vector3& W) -> Vector3
                {
                    Vector3 ref = abs(W.dot(Vector3(1, 0, 0))) < 0.99 ?
                                      Vector3(1, 0, 0) :
                                      Vector3(0, 1, 0);

                    Vector3 U = ref.cross(W).normalized();
                    Vector3 V = W.cross(U).normalized();

                    return V;
                };

                for(auto&& [i, e] : enumerate(Es))
                {
                    Vector2i geo_id  = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    Vector3 P0  = Ps[e[0]];
                    Vector3 P1  = Ps[e[1]];
                    Vector3 mid = (P0 + P1) / 2;
                    Vector3 Dir = (P1 - P0);

                    UIPC_ASSERT(Dir.squaredNorm() > 1e-24,
                                R"(AffineBodyDrivingRevoluteJoint: Edge with zero length detected,
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

                    Vector3 HalfAxis = Dir.normalized() / 2;

                    // Re-define P0 and P1 to be symmetric around the mid-point
                    P0 = mid - HalfAxis;
                    P1 = mid + HalfAxis;

                    Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                         info.body_id(geo_id(1), inst_id(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                    auto right_sc = info.body_geo(geo_slots, geo_id(1));

                    UIPC_ASSERT(inst_id(0) >= 0
                                    && inst_id(0) < static_cast<IndexT>(
                                           left_sc->instances().size()),
                                "AffineBodyDrivingRevoluteJoint: Left instance ID {} is out of range [0, {})",
                                inst_id(0),
                                left_sc->instances().size());
                    UIPC_ASSERT(inst_id(1) >= 0
                                    && inst_id(1) < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyDrivingRevoluteJoint: Right instance ID {} is out of range [0, {})",
                                inst_id(1),
                                right_sc->instances().size());

                    Vector3 UnitE = (P0 - P1).normalized();
                    // normal
                    Vector3 normal = toNormal(UnitE);
                    Vector3 vec    = normal.cross(UnitE).normalized();

                    Transform LT{left_sc->transforms().view()[inst_id(0)]};
                    Transform RT{right_sc->transforms().view()[inst_id(1)]};

                    // axis
                    Vector6 rest_axis;
                    rest_axis.segment<3>(0) = LT.rotation().inverse() * vec;
                    rest_axis.segment<3>(3) = RT.rotation().inverse() * vec;
                    rest_axis_list.push_back(rest_axis);

                    // normal
                    Vector6 rest_normal;
                    rest_normal.segment<3>(0) = LT.rotation().inverse() * normal;
                    rest_normal.segment<3>(3) = RT.rotation().inverse() * normal;
                    rest_normals_list.push_back(rest_normal);
                }
                joint_offset++;
            });

        h_geo_joint_offsets_counts.scan();

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_rest_axis.resize(rest_axis_list.size());
        std::ranges::move(rest_axis_list, h_rest_axis.begin());

        h_rest_normals.resize(rest_normals_list.size());
        std::ranges::move(rest_normals_list, h_rest_normals.begin());

        h_is_constrained.resize(is_constrained_list.size());
        std::ranges::copy(is_constrained_list, h_is_constrained.begin());

        h_strength_ratios.resize(strength_ratios_list.size());
        std::ranges::copy(strength_ratios_list, h_strength_ratios.begin());

        h_is_passive.resize(is_passive_list.size());
        std::ranges::copy(is_passive_list, h_is_passive.begin());

        h_init_angles.resize(init_angles_list.size());
        std::ranges::copy(init_angles_list, h_init_angles.begin());

        h_aim_angles.resize(aim_angles_list.size());
        std::ranges::copy(aim_angles_list, h_aim_angles.begin());

        body_ids.copy_from(h_body_ids);
        rest_axis.copy_from(h_rest_axis);
        rest_normals.copy_from(h_rest_normals);
        is_constrained.copy_from(h_is_constrained);
        strength_ratios.copy_from(h_strength_ratios);
        is_passive.copy_from(h_is_passive);
        init_angles.copy_from(h_init_angles);
        aim_angles.copy_from(h_aim_angles);
        current_angles = init_angles;
    }


    void do_step(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto  geo_slots       = world().scene().geometries();
        SizeT geo_joint_index = 0;

        is_constrained_changed  = false;
        strength_ratios_changed = false;
        is_passive_changed      = false;
        aim_angles_changed      = false;

        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyDrivingRevoluteJoint: Geometry must be a simplicial complex");

                auto joint_geo_id = I.geo_info().geo_id;
                auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];

                UIPC_ASSERT(sc->edges().size() == count,
                            "AffineBodyDrivingRevoluteJoint: Geometry edges size {} mismatch with joint count {}",
                            sc->edges().size(),
                            count);

                auto is_constrained = sc->edges().find<IndexT>(builtin::is_constrained);
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_constrained' attribute on `edges`");
                {
                    auto is_constrained_view = is_constrained->view();
                    auto dst = span{h_is_constrained}.subspan(offset, count);
                    std::ranges::copy(is_constrained_view, dst.begin());
                    is_constrained_changed = true;
                }

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_passive' attribute on `edges`")
                {
                    auto is_passive_view = is_passive->view();
                    auto dst = span{h_is_passive}.subspan(offset, count);
                    std::ranges::copy(is_passive_view, dst.begin());
                    is_passive_changed = true;
                }

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                {
                    auto strength_ratios_view = strength_ratios->view();
                    auto dst = span{h_strength_ratios}.subspan(offset, count);
                    std::ranges::copy(strength_ratios_view, dst.begin());
                    strength_ratios_changed = true;
                }

                auto aim_angles = sc->edges().find<Float>("aim_angle");
                UIPC_ASSERT(aim_angles, "AffineBodyDrivingRevoluteJoint: Geometry must have 'aim_angles' attribute on `edges`");
                {
                    auto aim_angles_view = aim_angles->view();
                    auto dst = span{h_aim_angles}.subspan(offset, count);
                    std::ranges::copy(aim_angles_view, dst.begin());
                    aim_angles_changed = true;
                }
                ++geo_joint_index;
            });

        if(aim_angles_changed)
            aim_angles.copy_from(h_aim_angles);
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

        current_angles.copy_to(h_current_angles);

        IndexT geo_joint_index = 0;

        this->for_each(geo_slots,
                       [&](geometry::Geometry& geo)
                       {
                           auto sc = geo.as<geometry::SimplicialComplex>();
                           UIPC_ASSERT(sc, "AffineBodyDrivingRevoluteJoint: Geometry must be a simplicial complex");

                           auto angle = sc->edges().find<Float>("angle");

                           if(angle)
                           {
                               auto angle_view = view(*angle);
                               auto [offset, count] =
                                   h_geo_joint_offsets_counts[geo_joint_index];
                               UIPC_ASSERT(angle_view.size() == count,
                                           "AffineBodyDrivingRevoluteJoint: angle attribute size {} mismatch with joint count {}",
                                           angle_view.size(),
                                           count);

                               auto dst = span{h_current_angles}.subspan(offset, count);
                               std::ranges::copy(dst, angle_view.begin());
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
        namespace DRJ = sym::affine_body_driving_revolute_joint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids     = body_ids.cviewer().name("body_ids"),
                    rest_axis    = rest_axis.cviewer().name("rest_axis"),
                    rest_normals = rest_normals.cviewer().name("rest_normals"),
                    is_constrained = is_constrained.cviewer().name("is_constrained"),
                    strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                    is_passive  = is_passive.cviewer().name("is_passive"),
                    init_angles = init_angles.cviewer().name("init_angles"),
                    aim_angles  = aim_angles.cviewer().name("aim_angles"),
                    current_angles = current_angles.cviewer().name("current_angles"),
                    qs = info.qs().cviewer().name("qs"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bids        = body_ids(I);
                       auto     constrained = is_constrained(I);
                       // disable driving effect
                       if(constrained == 0)
                       {
                           Es(I) = 0.0;
                           return;
                       }

                       auto  passive = is_passive(I);
                       Float kappa   = strength_ratios(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());
                       auto aim_angle = aim_angles(I);
                       if(passive == 1)
                       {
                           // resist external forces passively
                           aim_angle = current_angles(I);
                       }

                       // mapping [min_angle, max_angle] to [min+init_angle, max+init_angle]
                       Float theta_tilde = aim_angle + init_angles(I);

                       Vector12 q_i        = qs(bids(0));
                       Vector12 q_j        = qs(bids(1));
                       Vector6  axis_bar   = rest_axis(I);
                       Vector6  normal_bar = rest_normals(I);

                       Vector12 F01_q;
                       DRJ::F01_q<Float>(F01_q,
                                         axis_bar.segment<3>(0),
                                         normal_bar.segment<3>(0),
                                         q_i,
                                         axis_bar.segment<3>(3),
                                         normal_bar.segment<3>(3),
                                         q_j);

                       // E = 1/2 * kappa * (sin(theta) cos(theta_tilde) - cos(theta) sin(theta_tilde))^2
                       Float E;
                       DRJ::E(E, kappa, F01_q, theta_tilde);
                       Es(I) = E;
                   });
    };


    void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override
    {
        using Vector24    = Vector<Float, 24>;
        using Matrix24x24 = Matrix<Float, 24, 24>;

        using namespace muda;
        namespace DRJ = sym::affine_body_driving_revolute_joint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                body_ids.size(),
                [body_ids     = body_ids.cviewer().name("body_ids"),
                 rest_axis    = rest_axis.cviewer().name("rest_axis"),
                 rest_normals = rest_normals.cviewer().name("rest_normals"),
                 is_constrained = is_constrained.cviewer().name("is_constrained"),
                 strength_ratios = strength_ratios.cviewer().name("strength_ratios"),
                 is_passive  = is_passive.cviewer().name("is_passive"),
                 init_angles = init_angles.cviewer().name("init_angles"),
                 aim_angles  = aim_angles.cviewer().name("aim_angles"),
                 current_angles = current_angles.cviewer().name("current_angles"),
                 qs          = info.qs().cviewer().name("qs"),
                 body_masses = info.body_masses().cviewer().name("body_masses"),
                 G12s        = info.gradients().viewer().name("G12s"),
                 H12x12s = info.hessians().viewer().name("H12x12s")] __device__(int I)
                {
                    Vector2i bids        = body_ids(I);
                    auto     constrained = is_constrained(I);
                    if(constrained == 0)
                    {
                        // no gradient and Hessian
                        DoubletVectorAssembler DVA{G12s};
                        DVA.segment<StencilSize>(I * StencilSize).write(bids, Vector24::Zero());

                        TripletMatrixAssembler TMA{H12x12s};
                        TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, Matrix24x24::Zero());
                        return;
                    }

                    auto passive = is_passive(I);
                    auto kappa =
                        strength_ratios(I)
                        * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());
                    auto aim_angle = aim_angles(I);
                    if(passive == 1)
                    {
                        // resist external forces passively
                        aim_angle = current_angles(I);
                    }

                    auto theta_tilde = aim_angle + init_angles(I);

                    Vector12 q_i = qs(bids(0));
                    Vector12 q_j = qs(bids(1));

                    const Vector6& axis_bar   = rest_axis(I);
                    const Vector6& normal_bar = rest_normals(I);

                    Vector12 F01_q;
                    DRJ::F01_q<Float>(F01_q,
                                      axis_bar.segment<3>(0),
                                      normal_bar.segment<3>(0),
                                      q_i,
                                      axis_bar.segment<3>(3),
                                      normal_bar.segment<3>(3),
                                      q_j);

                    // G12s
                    Vector12 G01;
                    DRJ::dEdF01<Float>(G01, kappa, F01_q, theta_tilde);
                    Vector24 J01T_G01;
                    DRJ::J01T_G01<Float>(J01T_G01,
                                         G01,
                                         axis_bar.segment<3>(0),
                                         normal_bar.segment<3>(0),
                                         axis_bar.segment<3>(3),
                                         normal_bar.segment<3>(3));

                    DoubletVectorAssembler DVA{G12s};
                    DVA.segment<StencilSize>(StencilSize * I).write(bids, J01T_G01);

                    // H12x12s
                    Matrix12x12 H01;
                    DRJ::ddEddF01<Float>(H01, kappa, F01_q, theta_tilde);
                    // H01 SPD to ensure the Hessian is positive definite
                    make_spd(H01);
                    Matrix24x24 J01T_H01_J01;
                    DRJ::J01T_H01_J01<Float>(J01T_H01_J01,
                                             H01,
                                             axis_bar.segment<3>(0),
                                             normal_bar.segment<3>(0),
                                             axis_bar.segment<3>(3),
                                             normal_bar.segment<3>(3));

                    TripletMatrixAssembler TMA{H12x12s};
                    TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, J01T_H01_J01);
                });
    };

    U64 get_uid() const noexcept override { return ConstraintUID; }


    BufferDump curr_angles_dump;

    bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_angles_dump.dump(fmt::format("{}current_angle.{}", path, frame),
                                     current_angles);
    }

    bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_angles_dump.load(fmt::format("{}current_angle.{}", path, frame));
    }

    void do_apply_recover(RecoverInfo& info) override
    {
        curr_angles_dump.apply_to(current_angles);
    }

    void do_clear_recover(RecoverInfo& info) override
    {
        curr_angles_dump.clean_up();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingRevoluteJoint);


class AffineBodyDrivingRevoluteJointTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyDrivingRevoluteJoint> driving_revolute_joint;
    SimSystemSlot<AffineBodyDynamics>             affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        driving_revolute_joint = require<AffineBodyDrivingRevoluteJoint>();
        affine_body_dynamics   = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override
    {
        // do nothing here
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace DRJ = sym::affine_body_driving_revolute_joint;
        auto& drj     = driving_revolute_joint;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(drj->body_ids.size(),
                   [body_ids  = drj->body_ids.cviewer().name("body_ids"),
                    rest_axis = drj->rest_axis.cviewer().name("rest_axis"),
                    rest_normals = drj->rest_normals.cviewer().name("rest_normals"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs"),
                    current_angles = drj->current_angles.viewer().name("current_angles"),
                    init_angles = drj->init_angles.cviewer().name("init_angles"),
                    PI = std::numbers::pi] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Vector12 q_i        = qs(bids(0));
                       Vector12 q_j        = qs(bids(1));
                       Vector6  axis_bar   = rest_axis(I);
                       Vector6  normal_bar = rest_normals(I);

                       Vector12 F01_q;
                       DRJ::F01_q<Float>(F01_q,
                                         axis_bar.segment<3>(0),
                                         normal_bar.segment<3>(0),
                                         q_i,
                                         axis_bar.segment<3>(3),
                                         normal_bar.segment<3>(3),
                                         q_j);

                       // Compute current angle
                       Float curr_angle;
                       DRJ::currAngle<Float>(curr_angle, F01_q);


                       Float total_angle = curr_angle - init_angles(I);
                       // map to [-pi, pi]
                       auto map2range = [=](Float angle) -> Float
                       {
                           if(angle > PI)
                           {
                               angle -= 2 * PI;
                           }
                           else if(angle < -PI)
                           {
                               angle += 2 * PI;
                           }
                           return angle;
                       };

                       current_angles(I) = map2range(total_angle);
                   });
    }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingRevoluteJointTimeIntegrator);
}  // namespace uipc::backend::cuda