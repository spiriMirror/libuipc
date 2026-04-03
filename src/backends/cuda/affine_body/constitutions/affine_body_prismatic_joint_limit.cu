#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constraints/external_articulation_constraint_function.h>
#include <affine_body/constitutions/joint_limit_penalty.h>
#include <affine_body/utils.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>
namespace uipc::backend::cuda
{
class AffineBodyPrismaticJointLimit final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 669;
    static constexpr U64   JointUID        = 20;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;

    vector<Vector2i> h_body_ids;
    vector<Vector6>  h_l_basis;
    vector<Vector6>  h_r_basis;
    vector<Vector24> h_ref_qs;
    vector<Float>    h_lowers;
    vector<Float>    h_uppers;
    vector<Float>    h_strengths;

    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6>  l_basis;
    muda::DeviceBuffer<Vector6>  r_basis;
    muda::DeviceBuffer<Vector24> ref_qs;
    muda::DeviceBuffer<Float>    lowers;
    muda::DeviceBuffer<Float>    uppers;
    muda::DeviceBuffer<Float>    strengths;

    static auto get_prismatic_basis(const geometry::SimplicialComplex* L,
                                    IndexT L_inst_id,
                                    const geometry::SimplicialComplex* R,
                                    IndexT R_inst_id,
                                    const geometry::SimplicialComplex* joint_mesh,
                                    IndexT joint_index)
    {
        auto topo_view = joint_mesh->edges().topo().view();
        auto pos_view  = joint_mesh->positions().view();

        Vector2i e = topo_view[joint_index];
        Vector3  t = pos_view[e[1]] - pos_view[e[0]];
        UIPC_ASSERT(t.squaredNorm() > 0.0,
                    "AffineBodyPrismaticJointLimit: joint edge {} has zero length; cannot compute prismatic basis",
                    joint_index);
        t         = t.normalized();
        Vector3 c = pos_view[e[0]];

        auto compute_ct_bar = [&](const geometry::SimplicialComplex* geo, IndexT inst_id) -> Vector6
        {
            UIPC_ASSERT(geo, "AffineBodyPrismaticJointLimit: geometry is null when computing basis");

            const Matrix4x4& trans = geo->transforms().view()[inst_id];
            Transform        T{trans};
            Matrix3x3        inv_rot = T.rotation().inverse();
            Vector6          ct_bar;
            ct_bar.segment<3>(0) = T.inverse() * c;
            ct_bar.segment<3>(3) = inv_rot * t;
            return ct_bar;
        };

        Vector6 L_ct_bar = compute_ct_bar(L, L_inst_id);
        Vector6 R_ct_bar = compute_ct_bar(R, R_inst_id);
        return std::tuple{L_ct_bar, R_ct_bar};
    }

    void do_build(BuildInfo& info) override {}

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        h_body_ids.clear();
        h_l_basis.clear();
        h_r_basis.clear();
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
                auto r_geo_id = sc->edges().find<IndexT>("r_geo_id");
                UIPC_ASSERT(r_geo_id, "AffineBodyPrismaticJointLimit requires `r_geo_id` attribute on edges");
                auto r_geo_id_view = r_geo_id->view();
                auto l_inst_id = sc->edges().find<IndexT>("l_inst_id");
                UIPC_ASSERT(l_inst_id, "AffineBodyPrismaticJointLimit requires `l_inst_id` attribute on edges");
                auto l_inst_id_view = l_inst_id->view();
                auto r_inst_id = sc->edges().find<IndexT>("r_inst_id");
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

                auto init_dist_attr = sc->edges().find<Float>("init_distance");
                UIPC_ASSERT(init_dist_attr, "AffineBodyPrismaticJointLimit requires `init_distance` attribute on edges");
                auto init_dist_view = init_dist_attr->view();

                auto edges = sc->edges().topo().view();

                auto l_pos0_attr = sc->edges().find<Vector3>("l_position0");
                auto l_pos1_attr = sc->edges().find<Vector3>("l_position1");
                auto r_pos0_attr = sc->edges().find<Vector3>("r_position0");
                auto r_pos1_attr = sc->edges().find<Vector3>("r_position1");
                bool use_local = l_pos0_attr && l_pos1_attr && r_pos0_attr && r_pos1_attr;

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

                    Vector2i bid = {
                        info.body_id(l_gid, l_iid),
                        info.body_id(r_gid, r_iid),
                    };

                    Vector6 lb, rb;
                    if(use_local)
                    {
                        Transform LT{left_sc->transforms().view()[l_iid]};
                        Transform RT{right_sc->transforms().view()[r_iid]};
                        Vector3 l_t = (LT.rotation() * (l_pos1_attr->view()[i] - l_pos0_attr->view()[i])).normalized();
                        Vector3 l_c = LT * l_pos0_attr->view()[i];
                        Vector3 r_t = (RT.rotation() * (r_pos1_attr->view()[i] - r_pos0_attr->view()[i])).normalized();
                        Vector3 r_c = RT * r_pos0_attr->view()[i];
                        lb.segment<3>(0) = LT.inverse() * l_c;
                        lb.segment<3>(3) = LT.rotation().inverse() * l_t;
                        rb.segment<3>(0) = RT.inverse() * r_c;
                        rb.segment<3>(3) = RT.rotation().inverse() * r_t;
                    }
                    else
                    {
                        std::tie(lb, rb) = get_prismatic_basis(
                            left_sc, l_iid, right_sc, r_iid, sc, i);
                    }

                    Vector24 ref;
                    ref.segment<12>(0) =
                        transform_to_q(left_sc->transforms().view()[l_iid]);
                    ref.segment<12>(12) =
                        transform_to_q(right_sc->transforms().view()[r_iid]);

                    h_body_ids.push_back(bid);
                    h_l_basis.push_back(lb);
                    h_r_basis.push_back(rb);
                    h_ref_qs.push_back(ref);
                    Float init_dist    = init_dist_view[i];
                    Float actual_lower = lower_view[i] + init_dist;
                    Float actual_upper = upper_view[i] + init_dist;
                    UIPC_ASSERT(actual_lower <= actual_upper,
                                "AffineBodyPrismaticJointLimit: requires `limit/lower + init_distance <= limit/upper + init_distance` on edge {}, but got lower={} upper={} init_distance={}",
                                i,
                                lower_view[i],
                                upper_view[i],
                                init_dist);
                    h_lowers.push_back(actual_lower);
                    h_uppers.push_back(actual_upper);
                    h_strengths.push_back(strength_view[i]);
                }
            });

        body_ids.copy_from(h_body_ids);
        l_basis.copy_from(h_l_basis);
        r_basis.copy_from(h_r_basis);
        ref_qs.copy_from(h_ref_qs);
        lowers.copy_from(h_lowers);
        uppers.copy_from(h_uppers);
        strengths.copy_from(h_strengths);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace EPJ = sym::external_prismatic_joint_constraint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids  = body_ids.cviewer().name("body_ids"),
                    l_basis   = l_basis.cviewer().name("l_basis"),
                    r_basis   = r_basis.cviewer().name("r_basis"),
                    ref_qs    = ref_qs.cviewer().name("ref_qs"),
                    lowers    = lowers.cviewer().name("lowers"),
                    uppers    = uppers.cviewer().name("uppers"),
                    strengths = strengths.cviewer().name("strengths"),
                    qs        = info.qs().cviewer().name("qs"),
                    q_prevs   = info.q_prevs().cviewer().name("q_prevs"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bid = body_ids(I);

                       Vector6  lb    = l_basis(I);
                       Vector6  rb    = r_basis(I);
                       Vector24 ref_q = ref_qs(I);

                       Vector12 qk      = qs(bid[0]);
                       Vector12 ql      = qs(bid[1]);
                       Vector12 q_prevk = q_prevs(bid[0]);
                       Vector12 q_prevl = q_prevs(bid[1]);
                       Vector12 q_refk  = ref_q.segment<12>(0);
                       Vector12 q_refl  = ref_q.segment<12>(12);

                       Float theta_prev = 0.0f;
                       EPJ::DeltaTheta<Float>(theta_prev, lb, q_prevk, q_refk, rb, q_prevl, q_refl);

                       Float delta = 0.0f;
                       EPJ::DeltaTheta<Float>(delta, lb, qk, q_prevk, rb, ql, q_prevl);

                       Float x        = theta_prev + delta;
                       Float lower    = lowers(I);
                       Float upper    = uppers(I);
                       Float strength = strengths(I);

                       Float E = joint_limit::eval_penalty_energy<Float>(x, lower, upper, strength);

                       Es(I) = E;
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(2 * body_ids.size());
        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * body_ids.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace EPJ      = sym::external_prismatic_joint_constraint;
        auto gradient_only = info.gradient_only();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids  = body_ids.cviewer().name("body_ids"),
                    l_basis   = l_basis.cviewer().name("l_basis"),
                    r_basis   = r_basis.cviewer().name("r_basis"),
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

                       Vector6  lb    = l_basis(I);
                       Vector6  rb    = r_basis(I);
                       Vector24 ref_q = ref_qs(I);

                       Vector12 qk      = qs(bid[0]);
                       Vector12 ql      = qs(bid[1]);
                       Vector12 q_prevk = q_prevs(bid[0]);
                       Vector12 q_prevl = q_prevs(bid[1]);
                       Vector12 q_refk  = ref_q.segment<12>(0);
                       Vector12 q_refl  = ref_q.segment<12>(12);

                       Float theta_prev = 0.0f;
                       EPJ::DeltaTheta<Float>(theta_prev, lb, q_prevk, q_refk, rb, q_prevl, q_refl);

                       Float delta = 0.0f;
                       EPJ::DeltaTheta<Float>(delta, lb, qk, q_prevk, rb, ql, q_prevl);

                       Float x        = theta_prev + delta;
                       Float lower    = lowers(I);
                       Float upper    = uppers(I);
                       Float strength = strengths(I);

                       Float dE_dx   = 0.0f;
                       Float d2E_dx2 = 0.0f;
                       joint_limit::eval_penalty_derivatives<Float>(
                           x, lower, upper, strength, dE_dx, d2E_dx2);

                       Vector24 dx_dq;
                       EPJ::dDeltaTheta_dQ<Float>(dx_dq, lb, qk, q_prevk, rb, ql, q_prevl);

                       Vector24               G = dE_dx * dx_dq;
                       DoubletVectorAssembler DVA{G12s};
                       DVA.segment<2>(2 * I).write(bid, G);

                       if(gradient_only)
                           return;

                       Matrix24x24 H = d2E_dx2 * (dx_dq * dx_dq.transpose());

                       if(dE_dx != 0.0f)
                       {
                           Vector12 F;
                           Vector12 F_prev;
                           EPJ::F<Float>(F, lb, qk, rb, ql);
                           EPJ::F<Float>(F_prev, lb, q_prevk, rb, q_prevl);

                           Matrix12x12 ddx_ddF;
                           EPJ::ddDeltaTheta_ddF(ddx_ddF, F, F_prev);

                           Matrix12x12 H_F = dE_dx * ddx_ddF;
                           make_spd(H_F);

                           Matrix24x24 JT_H_J;
                           EPJ::JT_H_J<Float>(JT_H_J, H_F, lb, rb, lb, rb);
                           H += JT_H_J;
                       }

                       TripletMatrixAssembler TMA{H12x12s};
                       TMA.half_block<2>(HalfHessianSize * I).write(bid, H);
                   });
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};

REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointLimit);
}  // namespace uipc::backend::cuda
