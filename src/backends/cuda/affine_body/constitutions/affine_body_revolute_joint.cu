#include <numbers>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <time_integrator/time_integrator.h>
#include <affine_body/inter_affine_body_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <affine_body/constitutions/joint_limit_penalty.h>
#include <uipc/common/enumerate.h>
#include <affine_body/utils.h>
#include <affine_body/affine_body_external_force_reporter.h>
#include <joint_dof_system/joint_dof_reporter.h>
#include <cuda_tool/cuda_tool.h>


namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;  // for eigen:: / view aliases used inside kernel bodies

    // Hoisted from the constitution classes below (AffineBodyRevoluteJoint,
    // AffineBodyDrivingRevoluteJoint, AffineBodyRevoluteJointLimit all define
    // StencilSize = 2 and HalfHessianSize = 3) so kernel bodies can reference
    // the same names verbatim.
    constexpr SizeT StencilSize     = 2;
    constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;
    using Matrix6x6   = Matrix<Float, 6, 6>;

    namespace RJ  = sym::affine_body_revolute_joint;
    namespace DRJ = sym::affine_body_driving_revolute_joint;
    namespace ERJ = sym::affine_body_revolute_joint_limit;

    __global__ void affine_body_revolute_joint_compute_current_angles_kernel(
        cuda_tool::CBufferView<Vector2i> body_ids,
        cuda_tool::CBufferView<Vector6>  l_basis,
        cuda_tool::CBufferView<Vector6>  r_basis,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::BufferView<Float>     current_angles,
        cuda_tool::CBufferView<Float>    init_angles,
        Float                            PI,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector2i bids = body_ids(I);

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));
        Vector6  lb  = l_basis(I);
        Vector6  rb  = r_basis(I);

        Float theta;
        compute_relative_angle(theta, lb, q_i, rb, q_j);

        // Unwrap the (-pi, pi] atan2 angle against the stored multi-turn angle; exact while per-frame rotation < pi.
        Float prev_rel = current_angles(I) - init_angles(I);
        Float rel      = unwrap_angle(theta, prev_rel);
        // Wrong winding is silent and never self-corrects; warn near the pi aliasing limit.
        if(::fabs(rel - prev_rel) > 0.9 * PI)
        {
            UIPC_KERNEL_WARN_WITH_LOCATION(
                "revolute joint %d: per-frame rotation %f rad is "
                "close to the pi unwrap limit; current_angle may "
                "alias onto a wrong turn",
                I,
                rel - prev_rel);
        }
        current_angles(I) = rel + init_angles(I);
        UIPC_KERNEL_ASSERT(::isfinite(current_angles(I)),
                           "current_angle is not finite: %f (theta=%f, prev_rel=%f)",
                           current_angles(I),
                           theta,
                           prev_rel);
    }

    __global__ void affine_body_revolute_joint_compute_energy_kernel(
        cuda_tool::CBufferView<Vector2i>            body_ids,
        cuda_tool::CBufferView<Vector12>            rest_positions,
        cuda_tool::CBufferView<Float>               strength_ratio,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::BufferView<Float>                Es,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector2i bids = body_ids(I);

        Float kappa = strength_ratio(I)
                      * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

        const Vector12& X_bar = rest_positions(I);

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));

        // qi0_bar, qi1_bar, qj0_bar, qj1_bar
        Vector3 qi0_bar = X_bar.segment<3>(0);
        Vector3 qi1_bar = X_bar.segment<3>(3);
        Vector3 qj0_bar = X_bar.segment<3>(6);
        Vector3 qj1_bar = X_bar.segment<3>(9);

        // Compute constraint violation in F-space
        Vector6 F;
        RJ::Faxis<Float>(F, qi0_bar, qi1_bar, q_i, qj0_bar, qj1_bar, q_j);

        // Compute energy: E = 0.5 * kappa * (||d0||^2 + ||d1||^2)
        Float E;
        RJ::Eaxis<Float>(E, kappa, F);
        Es(I) = E;
    }

    __global__ void affine_body_revolute_joint_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector2i>            body_ids,
        cuda_tool::CBufferView<Vector12>            rest_positions,
        cuda_tool::CBufferView<Float>               strength_ratio,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::DoubletVectorView<Float, 12>     G12s,
        cuda_tool::TripletMatrixView<Float, 12>     H12x12s,
        bool                                        gradient_only,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector2i        bids  = body_ids(I);
        const Vector12& X_bar = rest_positions(I);

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));

        // Extract rest positions
        Vector3 qi0_bar = X_bar.segment<3>(0);
        Vector3 qi1_bar = X_bar.segment<3>(3);
        Vector3 qj0_bar = X_bar.segment<3>(6);
        Vector3 qj1_bar = X_bar.segment<3>(9);

        Float K = strength_ratio(I)
                  * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

        // Compute constraint violation in F-space
        Vector6 F;
        RJ::Faxis<Float>(F, qi0_bar, qi1_bar, q_i, qj0_bar, qj1_bar, q_j);

        // Compute gradient in F-space
        Vector6 dEdF;
        RJ::dEaxisdFaxis<Float>(dEdF, K, F);

        // Map gradient back to ABD space: G24 = J^T * dEdF
        Vector24 G24;
        RJ::JaxisT_Gaxis<Float>(G24, dEdF, qi0_bar, qi1_bar, qj0_bar, qj1_bar);

        // Fill Body Gradient
        DoubletVectorAssembler DVA{G12s};
        DVA.segment<StencilSize>(StencilSize * I).write(bids, G24);
        if(gradient_only)
        {
            return;
        }
        // Fill Body Hessian
        Matrix6x6 ddEddF;
        RJ::ddEaxisddFaxis<Float>(ddEddF, K, F);
        make_spd(ddEddF);

        // Map Hessian back to ABD space: H24 = J^T * ddEddF * J
        Matrix24x24 H24;
        RJ::JaxisT_Haxis_Jaxis<Float>(H24, ddEddF, qi0_bar, qi1_bar, qj0_bar, qj1_bar);

        TripletMatrixAssembler TMA{H12x12s};
        TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, H24);
    }

    __global__ void affine_body_driving_revolute_joint_compute_energy_kernel(
        cuda_tool::CBufferView<Vector2i>            body_ids,
        cuda_tool::CBufferView<Vector6>             l_basis,
        cuda_tool::CBufferView<Vector6>             r_basis,
        cuda_tool::CBufferView<IndexT>              is_constrained,
        cuda_tool::CBufferView<Float>               strength_ratios,
        cuda_tool::CBufferView<IndexT>              is_passive,
        cuda_tool::CBufferView<Float>               init_angles,
        cuda_tool::CBufferView<Float>               aim_angles,
        cuda_tool::CBufferView<Float>               current_angles,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::BufferView<Float>                Es,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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
                      * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());
        auto aim_angle = aim_angles(I);
        if(passive == 1)
        {
            // resist external forces passively
            aim_angle = current_angles(I);
        }

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));
        Vector6  lb  = l_basis(I);
        Vector6  rb  = r_basis(I);

        Vector12 F01_q;
        DRJ::F01_q<Float>(
            F01_q, lb.segment<3>(0), lb.segment<3>(3), q_i, rb.segment<3>(0), rb.segment<3>(3), q_j);

        // Fold the frame-start unwrap shift into theta_tilde (gradient unaffected; see driving_theta_tilde).
        Float theta_tilde =
            driving_theta_tilde(F01_q, aim_angle, init_angles(I), current_angles(I));

        Float E;
        DRJ::E(E, kappa, F01_q, theta_tilde);
        Es(I) = E;
    }

    __global__ void affine_body_driving_revolute_joint_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector2i>            body_ids,
        cuda_tool::CBufferView<Vector6>             l_basis,
        cuda_tool::CBufferView<Vector6>             r_basis,
        cuda_tool::CBufferView<IndexT>              is_constrained,
        cuda_tool::CBufferView<Float>               strength_ratios,
        cuda_tool::CBufferView<IndexT>              is_passive,
        cuda_tool::CBufferView<Float>               init_angles,
        cuda_tool::CBufferView<Float>               aim_angles,
        cuda_tool::CBufferView<Float>               current_angles,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::DoubletVectorView<Float, 12>     G12s,
        cuda_tool::TripletMatrixView<Float, 12>     H12x12s,
        bool                                        gradient_only,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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
        auto kappa   = strength_ratios(I)
                     * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());
        auto aim_angle = aim_angles(I);
        if(passive == 1)
        {
            // resist external forces passively
            aim_angle = current_angles(I);
        }

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));

        Vector6 lb = l_basis(I);
        Vector6 rb = r_basis(I);

        Vector12 F01_q;
        DRJ::F01_q<Float>(
            F01_q, lb.segment<3>(0), lb.segment<3>(3), q_i, rb.segment<3>(0), rb.segment<3>(3), q_j);

        // Same unwrap as do_compute_energy (must be bit-identical).
        Float theta_tilde =
            driving_theta_tilde(F01_q, aim_angle, init_angles(I), current_angles(I));

        // G12s
        Vector12 G01;
        DRJ::dEdF01<Float>(G01, kappa, F01_q, theta_tilde);
        Vector24 J01T_G01;
        DRJ::J01T_G01<Float>(J01T_G01,
                             G01,
                             lb.segment<3>(0),
                             lb.segment<3>(3),
                             rb.segment<3>(0),
                             rb.segment<3>(3));

        DoubletVectorAssembler DVA{G12s};
        DVA.segment<StencilSize>(StencilSize * I).write(bids, J01T_G01);

        if(gradient_only)
        {
            return;
        }
        // H12x12s
        Matrix12x12 H01;
        DRJ::ddEddF01<Float>(H01, kappa, F01_q, theta_tilde);
        // H01 SPD to ensure the Hessian is positive definite
        make_spd(H01);
        Matrix24x24 J01T_H01_J01;
        DRJ::J01T_H01_J01<Float>(J01T_H01_J01,
                                 H01,
                                 lb.segment<3>(0),
                                 lb.segment<3>(3),
                                 rb.segment<3>(0),
                                 rb.segment<3>(3));

        TripletMatrixAssembler TMA{H12x12s};
        TMA.half_block<StencilSize>(HalfHessianSize * I).write(bids, J01T_H01_J01);
    }

    __global__ void affine_body_revolute_joint_external_force_step_kernel(
        cuda_tool::BufferView<Vector12>  external_forces,
        cuda_tool::CBufferView<Vector2i> body_ids,
        cuda_tool::CBufferView<Float>    torques,
        cuda_tool::CBufferView<Vector12> rest_positions,
        cuda_tool::CBufferView<IndexT>   constrained_flags,
        cuda_tool::CBufferView<Vector12> qs,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(constrained_flags(i) == 0)
            return;

        Vector2i bids = body_ids(i);
        Float    tau  = torques(i);

        const Vector12& X_bar = rest_positions(i);

        Vector12 q_i = qs(bids(0));
        Vector12 q_j = qs(bids(1));

        // position direction of the applied torque on body j (right-hand rule)
        ABDJacobi e_bar[2] = {ABDJacobi{X_bar.segment<3>(3) - X_bar.segment<3>(0)},
                              ABDJacobi{X_bar.segment<3>(9) - X_bar.segment<3>(6)}};
        Vector3 e_i = e_bar[0].vec_x(q_i).normalized();
        Vector3 e_j = e_bar[1].vec_x(q_j).normalized();

        // symmetrize to avoid numerical issues when the joint is near singularity
        e_j = 0.5 * (e_j + e_i);
        e_i = -e_j;

        // affine-body rotational DOF (virtual-work form),
        // see torque_to_F in affine_body/utils.h
        Vector12 F_i = torque_to_F(tau, e_i, q_i);
        Vector12 F_j = torque_to_F(tau, e_j, q_j);

        eigen::atomic_add(external_forces(bids(0)), F_i);
        eigen::atomic_add(external_forces(bids(1)), F_j);
    }

    __global__ void affine_body_revolute_joint_limit_compute_energy_kernel(
        cuda_tool::CBufferView<Vector2i> body_ids,
        cuda_tool::CBufferView<Vector6>  l_basis,
        cuda_tool::CBufferView<Vector6>  r_basis,
        cuda_tool::CBufferView<Float>    init_angles,
        cuda_tool::CBufferView<Float>    current_angles,
        cuda_tool::CBufferView<Float>    lowers,
        cuda_tool::CBufferView<Float>    uppers,
        cuda_tool::CBufferView<Float>    strengths,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::CBufferView<Vector12> q_prevs,
        cuda_tool::BufferView<Float>     Es,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector2i bid = body_ids(I);

        Vector6 lb = l_basis(I);
        Vector6 rb = r_basis(I);

        Vector12 qk      = qs(bid[0]);
        Vector12 ql      = qs(bid[1]);
        Vector12 q_prevk = q_prevs(bid[0]);
        Vector12 q_prevl = q_prevs(bid[1]);

        Float init_a = init_angles(I);

        // Unwrapped relative angle at the last committed state; valid past +-pi and across turns.
        Float theta_prev = current_angles(I) - init_a;

        Float delta = 0.0f;
        ERJ::DeltaTheta<Float>(delta, lb, qk, q_prevk, rb, ql, q_prevl);

        Float x        = theta_prev + delta;
        Float lower    = lowers(I) - init_a;
        Float upper    = uppers(I) - init_a;
        Float strength = strengths(I);

        Float E = joint_limit::eval_penalty_energy<Float>(x, lower, upper, strength);

        Es(I) = E;
    }

    __global__ void affine_body_revolute_joint_limit_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector2i>        body_ids,
        cuda_tool::CBufferView<Vector6>         l_basis,
        cuda_tool::CBufferView<Vector6>         r_basis,
        cuda_tool::CBufferView<Float>           init_angles,
        cuda_tool::CBufferView<Float>           current_angles,
        cuda_tool::CBufferView<Float>           lowers,
        cuda_tool::CBufferView<Float>           uppers,
        cuda_tool::CBufferView<Float>           strengths,
        cuda_tool::CBufferView<Vector12>        qs,
        cuda_tool::CBufferView<Vector12>        q_prevs,
        cuda_tool::DoubletVectorView<Float, 12> G12s,
        cuda_tool::TripletMatrixView<Float, 12> H12x12s,
        bool                                    gradient_only,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector2i bid = body_ids(I);

        Vector6 lb = l_basis(I);
        Vector6 rb = r_basis(I);

        Vector12 qk      = qs(bid[0]);
        Vector12 ql      = qs(bid[1]);
        Vector12 q_prevk = q_prevs(bid[0]);
        Vector12 q_prevl = q_prevs(bid[1]);

        Float init_a = init_angles(I);

        // Same unwrapped frame-start angle as do_compute_energy.
        Float theta_prev = current_angles(I) - init_a;

        Float delta = 0.0f;
        ERJ::DeltaTheta<Float>(delta, lb, qk, q_prevk, rb, ql, q_prevl);

        Float x        = theta_prev + delta;
        Float lower    = lowers(I) - init_a;
        Float upper    = uppers(I) - init_a;
        Float strength = strengths(I);

        Float dE_dx   = 0.0f;
        Float d2E_dx2 = 0.0f;
        joint_limit::eval_penalty_derivatives<Float>(x, lower, upper, strength, dE_dx, d2E_dx2);

        Vector24 dx_dq;
        ERJ::dDeltaTheta_dQ<Float>(dx_dq, lb, qk, q_prevk, rb, ql, q_prevl);

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
            ERJ::F<Float>(F, lb, qk, rb, ql);
            ERJ::F<Float>(F_prev, lb, q_prevk, rb, q_prevl);

            Matrix12x12 ddx_ddF;
            ERJ::ddDeltaTheta_ddF(ddx_ddF, F, F_prev);

            Matrix12x12 H_F = dE_dx * ddx_ddF;
            make_spd(H_F);

            Matrix24x24 JT_H_J;
            ERJ::JT_H_J<Float>(JT_H_J, H_F, lb, rb, lb, rb);
            H += JT_H_J;
        }

        TripletMatrixAssembler TMA{H12x12s};
        TMA.half_block<2>(HalfHessianSize * I).write(bid, H);
    }
}  // namespace

class AffineBodyRevoluteJoint final : public InterAffineBodyConstitution
{
  public:
    using InterAffineBodyConstitution::InterAffineBodyConstitution;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    static constexpr SizeT StencilSize     = 2;

    static constexpr U64 ConstitutionUID = 18;


    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    vector<Vector2i> h_body_ids;
    // [    body0   |   body1    ]
    // [    x0, x1  |   x2, x3   ]
    vector<Vector12> h_rest_positions;
    vector<Float>    h_strength_ratio;

    // Angle tracking (for all revolute joints)
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;
    vector<Vector6>               h_l_basis;  // [n_L, b_L] per joint
    vector<Vector6>               h_r_basis;  // [n_R, b_R] per joint
    vector<Float>                 h_init_angles;
    vector<Float>                 h_current_angles;
    vector<Float>                 h_adopted_angles;

    cuda_tool::DeviceBuffer<Vector2i> body_ids;
    cuda_tool::DeviceBuffer<Vector12> rest_positions;
    cuda_tool::DeviceBuffer<Float>    strength_ratio;

    cuda_tool::DeviceBuffer<Vector6> l_basis;  // [n_L, b_L] per joint
    cuda_tool::DeviceBuffer<Vector6> r_basis;  // [n_R, b_R] per joint
    cuda_tool::DeviceBuffer<Float>   init_angles;
    cuda_tool::DeviceBuffer<Float>   current_angles;

    BufferDump curr_angles_dump;


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
        list<Vector12> rest_positions_list;
        list<Float>    strength_ratio_list;
        list<Vector6>  l_basis_list;
        list<Vector6>  r_basis_list;
        list<Float>    init_angles_list;

        IndexT geo_index = 0;
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

                h_geo_joint_offsets_counts.counts()[geo_index] = sc->edges().size();

                auto l_geo_id = sc->edges().find<IndexT>("l_geo_id");
                UIPC_ASSERT(l_geo_id, "AffineBodyRevoluteJoint: Geometry must have 'l_geo_id' attribute on `edges`");
                auto l_geo_id_view = l_geo_id->view();

                auto r_geo_id = sc->edges().find<IndexT>("r_geo_id");
                UIPC_ASSERT(r_geo_id, "AffineBodyRevoluteJoint: Geometry must have 'r_geo_id' attribute on `edges`");
                auto r_geo_id_view = r_geo_id->view();

                auto l_inst_id = sc->edges().find<IndexT>("l_inst_id");
                UIPC_ASSERT(l_inst_id, "AffineBodyRevoluteJoint: Geometry must have 'l_inst_id' attribute on `edges`");
                auto l_inst_id_view = l_inst_id->view();

                auto r_inst_id = sc->edges().find<IndexT>("r_inst_id");
                UIPC_ASSERT(r_inst_id, "AffineBodyRevoluteJoint: Geometry must have 'r_inst_id' attribute on `edges`");
                auto r_inst_id_view = r_inst_id->view();

                auto strength_ratio = sc->edges().find<Float>("strength_ratio");
                UIPC_ASSERT(strength_ratio, "AffineBodyRevoluteJoint: Geometry must have 'strength_ratio' attribute on `edges`");
                auto strength_ratio_view = strength_ratio->view();

                auto init_angle_attr = sc->edges().find<Float>("init_angle");
                UIPC_ASSERT(init_angle_attr, "AffineBodyRevoluteJoint: Geometry must have 'init_angle' attribute on `edges`");
                auto init_angle_view = init_angle_attr->view();

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
                                "AffineBodyRevoluteJoint: Left instance ID {} is out of range [0, {})",
                                l_iid,
                                left_sc->instances().size());
                    UIPC_ASSERT(r_iid >= 0
                                    && r_iid < static_cast<IndexT>(
                                           right_sc->instances().size()),
                                "AffineBodyRevoluteJoint: Right instance ID {} is out of range [0, {})",
                                r_iid,
                                right_sc->instances().size());

                    Transform LT{left_sc->transforms().view()[l_iid]};
                    Transform RT{right_sc->transforms().view()[r_iid]};

                    Matrix3x3 L_inv_rot = LT.rotation().inverse();
                    Matrix3x3 R_inv_rot = RT.rotation().inverse();

                    Vector12 rest_pos;
                    Vector3  t;

                    if(use_local)
                    {
                        rest_pos.segment<3>(0) = l_pos0_attr->view()[i];
                        rest_pos.segment<3>(3) = l_pos1_attr->view()[i];
                        rest_pos.segment<3>(6) = r_pos0_attr->view()[i];
                        rest_pos.segment<3>(9) = r_pos1_attr->view()[i];

                        t = LT.rotation()
                            * (l_pos1_attr->view()[i] - l_pos0_attr->view()[i]);
                    }
                    else
                    {
                        Vector3 P0 = Ps[e[0]];
                        Vector3 P1 = Ps[e[1]];
                        t          = P1 - P0;

                        rest_pos.segment<3>(0) = LT.inverse() * P0;
                        rest_pos.segment<3>(3) = LT.inverse() * P1;
                        rest_pos.segment<3>(6) = RT.inverse() * P0;
                        rest_pos.segment<3>(9) = RT.inverse() * P1;
                    }
                    UIPC_ASSERT(t.squaredNorm() > 1e-24,
                                R"(AffineBodyRevoluteJoint: Edge with zero length detected,
Joint GeometryID = {},
LinkGeoIDs       = ({}, {}),
LinkInstIDs      = ({}, {}),
Edge             = ({}, {}))",
                                joint_geo_id,
                                l_gid,
                                r_gid,
                                l_iid,
                                r_iid,
                                e(0),
                                e(1));

                    rest_positions_list.push_back(rest_pos);

                    Vector3 n, b;
                    orthonormal_basis(t, n, b);
                    // Storage layout: [n, b] per body
                    Vector6 lb;
                    lb.segment<3>(0) = L_inv_rot * n;
                    lb.segment<3>(3) = L_inv_rot * b;
                    l_basis_list.push_back(lb);

                    Vector6 rb;
                    rb.segment<3>(0) = R_inv_rot * n;
                    rb.segment<3>(3) = R_inv_rot * b;
                    r_basis_list.push_back(rb);

                    init_angles_list.push_back(init_angle_view[i]);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
                ++geo_index;
            });

        h_geo_joint_offsets_counts.scan();

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_rest_positions.resize(rest_positions_list.size());
        std::ranges::move(rest_positions_list, h_rest_positions.begin());

        h_strength_ratio.resize(strength_ratio_list.size());
        std::ranges::move(strength_ratio_list, h_strength_ratio.begin());

        h_l_basis.resize(l_basis_list.size());
        std::ranges::move(l_basis_list, h_l_basis.begin());

        h_r_basis.resize(r_basis_list.size());
        std::ranges::move(r_basis_list, h_r_basis.begin());

        h_init_angles.resize(init_angles_list.size());
        std::ranges::copy(init_angles_list, h_init_angles.begin());

        // Seed the persistent unwrapped angle with init_angles (zero relative rotation).
        h_current_angles = h_init_angles;

        body_ids.copy_from(h_body_ids);
        rest_positions.copy_from(h_rest_positions);
        strength_ratio.copy_from(h_strength_ratio);
        l_basis.copy_from(h_l_basis);
        r_basis.copy_from(h_r_basis);
        init_angles.copy_from(h_init_angles);
        current_angles.copy_from(h_current_angles);

        compute_current_angles();
        write_scene();
    }

    void compute_current_angles()
    {
        using namespace cuda_tool;

        auto k = affine_body_revolute_joint_compute_current_angles_kernel;
        int  n = (int)body_ids.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                body_ids.cview(),
                l_basis.cview(),
                r_basis.cview(),
                affine_body_dynamics->qs(),
                current_angles.view(),
                init_angles.cview(),
                std::numbers::pi,
                n);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());  // one energy per joint
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;
        namespace RJ = sym::affine_body_revolute_joint;
        auto k       = affine_body_revolute_joint_compute_energy_kernel;
        int  n       = (int)body_ids.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                body_ids.cview(),
                rest_positions.cview(),
                strength_ratio.cview(),
                info.body_masses(),
                info.qs(),
                info.energies(),
                n);
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
        using namespace cuda_tool;
        using Vector24    = Vector<Float, 24>;
        using Matrix24x24 = Matrix<Float, 24, 24>;
        using Matrix6x6   = Matrix<Float, 6, 6>;

        namespace RJ       = sym::affine_body_revolute_joint;
        auto gradient_only = info.gradient_only();

        auto k = affine_body_revolute_joint_compute_gradient_hessian_kernel;
        int  n = (int)body_ids.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                body_ids.cview(),
                rest_positions.cview(),
                strength_ratio.cview(),
                info.body_masses(),
                info.qs(),
                info.gradients(),
                info.hessians(),
                gradient_only,
                n);
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
                           UIPC_ASSERT(sc, "AffineBodyRevoluteJoint: Geometry must be a simplicial complex");

                           auto angle = sc->edges().find<Float>("angle");

                           if(angle)
                           {
                               auto angle_view = view(*angle);
                               auto [offset, count] =
                                   h_geo_joint_offsets_counts[geo_joint_index];
                               UIPC_ASSERT(angle_view.size() == count,
                                           "AffineBodyRevoluteJoint: angle attribute size {} mismatch with joint count {}",
                                           angle_view.size(),
                                           count);

                               auto src = span{h_current_angles}.subspan(offset, count);
                               std::ranges::copy(src, angle_view.begin());
                           }

                           ++geo_joint_index;
                       });
    }

    void adopt_scene_angles()
    {
        auto geo_slots = world().scene().geometries();

        // A mismatch vs h_current_angles (last published) means a user-authored angle (reset/teleport); overlay it, the following resync snaps it to the pose.
        current_angles.copy_to(h_adopted_angles);

        IndexT geo_joint_index = 0;
        bool   adopted         = false;

        this->for_each(geo_slots,
                       [&](geometry::Geometry& geo)
                       {
                           auto sc = geo.as<geometry::SimplicialComplex>();
                           UIPC_ASSERT(sc, "AffineBodyRevoluteJoint: Geometry must be a simplicial complex");

                           auto angle = sc->edges().find<Float>("angle");
                           if(angle)
                           {
                               auto angle_view = angle->view();
                               auto [offset, count] =
                                   h_geo_joint_offsets_counts[geo_joint_index];
                               UIPC_ASSERT(angle_view.size() == count,
                                           "AffineBodyRevoluteJoint: angle attribute size {} mismatch with joint count {}",
                                           angle_view.size(),
                                           count);

                               for(IndexT i = 0; i < count; ++i)
                               {
                                   Float user_angle = angle_view[i];
                                   if(user_angle != h_current_angles[offset + i])
                                   {
                                       h_adopted_angles[offset + i] = user_angle;
                                       adopted = true;
                                   }
                               }
                           }

                           ++geo_joint_index;
                       });

        if(adopted)
        {
            current_angles.copy_from(h_adopted_angles);
        }
    }

    bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_angles_dump.dump(fmt::format("{}rj_current_angle.{}", path, frame),
                                     current_angles);
    }

    bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return curr_angles_dump.load(fmt::format("{}rj_current_angle.{}", path, frame));
    }

    void do_apply_recover(RecoverInfo& info) override
    {
        curr_angles_dump.apply_to(current_angles);
    }

    void do_clear_recover(RecoverInfo& info) override
    {
        curr_angles_dump.clean_up();
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};

REGISTER_SIM_SYSTEM(AffineBodyRevoluteJoint);

class AffineBodyRevoluteJointDofReporter final : public JointDofReporter
{
  public:
    using JointDofReporter::JointDofReporter;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;

    void do_build(BuildInfo& info) override
    {
        revolute_joint = require<AffineBodyRevoluteJoint>();
    }

    void do_update_dof_attributes(UpdateDofAttributesInfo& info) override
    {
        // Adopt user-authored angles (reset/teleport winding a > pi jump can't unwrap), resync current_angles to the new pose, then republish.
        revolute_joint->adopt_scene_angles();
        revolute_joint->compute_current_angles();
        revolute_joint->write_scene();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointDofReporter);

class AffineBodyRevoluteJointTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;
    SimSystemSlot<AffineBodyDynamics>      affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        revolute_joint       = require<AffineBodyRevoluteJoint>();
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override
    {
        // do nothing here
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        revolute_joint->compute_current_angles();
    }
};
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointTimeIntegrator);

class AffineBodyDrivingRevoluteJoint : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;

    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    static constexpr SizeT StencilSize     = 2;

    static constexpr U64 ConstraintUID = 19;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;

    // Host
    // Note: body_ids / l_basis / r_basis / init_angles are reused directly
    // from AffineBodyRevoluteJoint under a strict index-alignment assumption
    // (see sanity check in do_init).
    OffsetCountCollection<IndexT> h_geo_joint_offsets_counts;

    vector<IndexT> h_is_constrained;
    vector<Float>  h_strength_ratios;
    vector<IndexT> h_is_passive;
    vector<Float>  h_aim_angles;

    // Device
    cuda_tool::DeviceBuffer<IndexT> is_constrained;
    cuda_tool::DeviceBuffer<Float>  strength_ratios;
    cuda_tool::DeviceBuffer<IndexT> is_passive;
    cuda_tool::DeviceBuffer<Float>  aim_angles;

    void do_build(BuildInfo& info) override
    {
        revolute_joint = require<AffineBodyRevoluteJoint>();
    }

    void do_init(InterAffineBodyAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        h_geo_joint_offsets_counts.resize(info.anim_inter_geo_infos().size());

        list<IndexT> is_constrained_list;
        list<Float>  strength_ratios_list;
        list<IndexT> is_passive_list;
        list<Float>  aim_angles_list;

        IndexT joint_offset = 0;
        info.for_each(
            geo_slots,
            [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
            {
                // check uid
                {
                    auto constraint_uids =
                        geo.meta().find<VectorXu64>(builtin::constraint_uids);
                    UIPC_ASSERT(constraint_uids, "AffineBodyDrivingRevoluteJoint: Geometry must have 'constraint_uids' attribute");
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
                                "AffineBodyDrivingRevoluteJoint: Geometry must have constraint UID {}",
                                ConstraintUID);
                }
                // check constitution uid
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

                // Driving-only attributes; shared attributes (body_ids, basis,
                // init_angles) are supplied by AffineBodyRevoluteJoint.
                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                std::ranges::copy(is_constrained->view(),
                                  std::back_inserter(is_constrained_list));

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                std::ranges::copy(strength_ratios->view(),
                                  std::back_inserter(strength_ratios_list));

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_passive' attribute on `edges`")
                std::ranges::copy(is_passive->view(), std::back_inserter(is_passive_list));

                auto aim_angles = sc->edges().find<Float>("aim_angle");
                UIPC_ASSERT(aim_angles, "AffineBodyDrivingRevoluteJoint: Geometry must have 'aim_angles' attribute on `edges`");
                std::ranges::copy(aim_angles->view(), std::back_inserter(aim_angles_list));

                joint_offset++;
            });

        h_geo_joint_offsets_counts.scan();

        h_is_constrained.resize(is_constrained_list.size());
        std::ranges::copy(is_constrained_list, h_is_constrained.begin());

        h_strength_ratios.resize(strength_ratios_list.size());
        std::ranges::copy(strength_ratios_list, h_strength_ratios.begin());

        h_is_passive.resize(is_passive_list.size());
        std::ranges::copy(is_passive_list, h_is_passive.begin());

        h_aim_angles.resize(aim_angles_list.size());
        std::ranges::copy(aim_angles_list, h_aim_angles.begin());

        is_constrained.copy_from(h_is_constrained);
        strength_ratios.copy_from(h_strength_ratios);
        is_passive.copy_from(h_is_passive);
        aim_angles.copy_from(h_aim_angles);

        // Sanity check: driving joint reuses body_ids/basis/init_angles from
        // AffineBodyRevoluteJoint via the shared index `I`. Both must therefore
        // enumerate exactly the same (geo, edge) sequence.
        UIPC_ASSERT(h_is_constrained.size() == revolute_joint->h_body_ids.size(),
                    "AffineBodyDrivingRevoluteJoint: joint count {} must equal "
                    "AffineBodyRevoluteJoint joint count {} (index alignment required)",
                    h_is_constrained.size(),
                    revolute_joint->h_body_ids.size());
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
                UIPC_ASSERT(sc, "AffineBodyDrivingRevoluteJoint: Geometry must be a simplicial complex");

                auto [offset, count] = h_geo_joint_offsets_counts[geo_joint_index];

                UIPC_ASSERT(sc->edges().size() == count,
                            "AffineBodyDrivingRevoluteJoint: Geometry edges size {} mismatch with joint count {}",
                            sc->edges().size(),
                            count);

                auto is_constrained = sc->edges().find<IndexT>("driving/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/is_constrained' attribute on `edges`");
                std::ranges::copy(is_constrained->view(),
                                  span{h_is_constrained}.subspan(offset, count).begin());

                auto is_passive = sc->edges().find<IndexT>("is_passive");
                UIPC_ASSERT(is_passive, "AffineBodyDrivingRevoluteJoint: Geometry must have 'is_passive' attribute on `edges`")
                std::ranges::copy(is_passive->view(),
                                  span{h_is_passive}.subspan(offset, count).begin());

                auto strength_ratios = sc->edges().find<Float>("driving/strength_ratio");
                UIPC_ASSERT(strength_ratios, "AffineBodyDrivingRevoluteJoint: Geometry must have 'driving/strength_ratio' attribute on `edges`")
                std::ranges::copy(strength_ratios->view(),
                                  span{h_strength_ratios}.subspan(offset, count).begin());

                auto aim_angles = sc->edges().find<Float>("aim_angle");
                UIPC_ASSERT(aim_angles, "AffineBodyDrivingRevoluteJoint: Geometry must have 'aim_angle' attribute on `edges`");
                std::ranges::copy(aim_angles->view(),
                                  span{h_aim_angles}.subspan(offset, count).begin());

                ++geo_joint_index;
            });

        aim_angles.copy_from(h_aim_angles);
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
        using namespace cuda_tool;
        namespace DRJ = sym::affine_body_driving_revolute_joint;

        auto k = affine_body_driving_revolute_joint_compute_energy_kernel;
        int  n = (int)is_constrained.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                revolute_joint->body_ids.cview(),
                revolute_joint->l_basis.cview(),
                revolute_joint->r_basis.cview(),
                is_constrained.cview(),
                strength_ratios.cview(),
                is_passive.cview(),
                revolute_joint->init_angles.cview(),
                aim_angles.cview(),
                revolute_joint->current_angles.cview(),
                info.qs(),
                info.body_masses(),
                info.energies(),
                n);
    };


    void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override
    {
        using Vector24    = Vector<Float, 24>;
        using Matrix24x24 = Matrix<Float, 24, 24>;

        using namespace cuda_tool;
        namespace DRJ = sym::affine_body_driving_revolute_joint;

        auto k = affine_body_driving_revolute_joint_compute_gradient_hessian_kernel;
        int n = (int)is_constrained.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                revolute_joint->body_ids.cview(),
                revolute_joint->l_basis.cview(),
                revolute_joint->r_basis.cview(),
                is_constrained.cview(),
                strength_ratios.cview(),
                is_passive.cview(),
                revolute_joint->init_angles.cview(),
                aim_angles.cview(),
                revolute_joint->current_angles.cview(),
                info.qs(),
                info.body_masses(),
                info.gradients(),
                info.hessians(),
                info.gradient_only(),
                n);
    };

    U64 get_uid() const noexcept override { return ConstraintUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyDrivingRevoluteJoint);

// ============================================================================
// AffineBodyRevoluteJointExternalForceConstraint
// Reuses body_ids, rest_positions, init_angles from AffineBodyRevoluteJoint.
// Only stores external-force-specific data (torques, is_constrained).
// ============================================================================

class AffineBodyRevoluteJointExternalForceConstraint final : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;
    static constexpr U64 ConstraintUID = 668;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;

    vector<Float>  h_torques;
    vector<IndexT> h_is_constrained;

    cuda_tool::DeviceBuffer<Float>  torques;
    cuda_tool::DeviceBuffer<IndexT> is_constrained;

    void do_build(BuildInfo& info) override
    {
        revolute_joint = require<AffineBodyRevoluteJoint>();
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
                UIPC_ASSERT(sc, "AffineBodyRevoluteJointExternalForceConstraint: geometry must be SimplicialComplex");

                auto is_constrained =
                    sc->edges().find<IndexT>("external_torque/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque/is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();

                auto external_torque = sc->edges().find<Float>("external_torque");
                UIPC_ASSERT(external_torque, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque' attribute on `edges`");
                auto external_torque_view = external_torque->view();

                for(SizeT i = 0; i < sc->edges().size(); ++i)
                {
                    h_is_constrained.push_back(is_constrained_view[i]);
                    h_torques.push_back(external_torque_view[i]);
                }
            });

        SizeT N = h_torques.size();
        if(N > 0)
        {
            torques.copy_from(h_torques);
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
                UIPC_ASSERT(sc, "AffineBodyRevoluteJointExternalForceConstraint: geometry must be SimplicialComplex");

                auto is_constrained =
                    sc->edges().find<IndexT>("external_torque/is_constrained");
                UIPC_ASSERT(is_constrained, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque/is_constrained' attribute on `edges`");
                auto is_constrained_view = is_constrained->view();

                auto external_torque = sc->edges().find<Float>("external_torque");
                UIPC_ASSERT(external_torque, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque' attribute on `edges`");
                auto external_torque_view = external_torque->view();

                auto Es = sc->edges().topo().view();
                for(auto&& [i, e] : enumerate(Es))
                {
                    h_is_constrained[offset] = is_constrained_view[i];
                    h_torques[offset]        = external_torque_view[i];
                    ++offset;
                }
            });

        SizeT N = h_torques.size();
        if(N > 0)
        {
            is_constrained.copy_from(h_is_constrained);
            torques.copy_from(h_torques);
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
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalForceConstraint);

// ============================================================================
// AffineBodyRevoluteJointExternalForce
// Applies external torques to revolute joints as tangential forces.
// ============================================================================

class AffineBodyRevoluteJointExternalForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 ReporterUID = 668;
    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyRevoluteJointExternalForceConstraint> constraint;

    void do_build(BuildInfo& info) override
    {
        constraint = require<AffineBodyRevoluteJointExternalForceConstraint>();
    }

    U64 get_uid() const noexcept override { return ReporterUID; }

    void do_init() override {}

    void do_step(ExternalForceInfo& info) override
    {
        SizeT torque_count = constraint->torques.size();
        if(torque_count == 0)
            return;

        auto abd = constraint->revolute_joint->affine_body_dynamics;

        using namespace cuda_tool;
        auto k = affine_body_revolute_joint_external_force_step_kernel;
        int  n = (int)torque_count;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.external_forces(),
            constraint->revolute_joint->body_ids.cview(),
            constraint->torques.cview(),
            constraint->revolute_joint->rest_positions.cview(),
            constraint->is_constrained.cview(),
            abd->qs(),
            n);
    }
};
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalForce);

// ============================================================================
// AffineBodyRevoluteJointLimit
// Penalty energy enforcing `limit/lower <= delta_angle <= limit/upper`.
// Reuses body_ids / l_basis / r_basis / init_angles / current_angles from
// AffineBodyRevoluteJoint (index-aligned). Only lowers, uppers, strengths
// are owned locally.
//
// Temporal contract: kernels read current_angles as the frame-start angle, so it must stay in sync with the qs snapshot copied into q_prevs; any new path that mutates qs between frames must re-sync it or the limit window shifts by a whole-frame delta.
// ============================================================================

class AffineBodyRevoluteJointLimit final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 670;
    static constexpr U64   JointUID = AffineBodyRevoluteJoint::ConstitutionUID;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;

    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    using Vector24    = Vector<Float, 24>;
    using Matrix24x24 = Matrix<Float, 24, 24>;

    SimSystemSlot<AffineBodyRevoluteJoint> revolute_joint;

    vector<Float> h_lowers;
    vector<Float> h_uppers;
    vector<Float> h_strengths;

    cuda_tool::DeviceBuffer<Float> lowers;
    cuda_tool::DeviceBuffer<Float> uppers;
    cuda_tool::DeviceBuffer<Float> strengths;

    void do_build(BuildInfo& info) override
    {
        revolute_joint = require<AffineBodyRevoluteJoint>();
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        h_lowers.clear();
        h_uppers.clear();
        h_strengths.clear();

        info.for_each(
            geo_slots,
            [&](geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                UIPC_ASSERT(uid && uid->view()[0] == JointUID,
                            "AffineBodyRevoluteJointLimit must be attached on base revolute joint geometry (UID={})",
                            JointUID);

                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyRevoluteJointLimit geometry must be SimplicialComplex");

                auto lower_attr = sc->edges().find<Float>("limit/lower");
                UIPC_ASSERT(lower_attr, "AffineBodyRevoluteJointLimit requires `limit/lower` attribute on edges");
                auto lower_view = lower_attr->view();

                auto upper_attr = sc->edges().find<Float>("limit/upper");
                UIPC_ASSERT(upper_attr, "AffineBodyRevoluteJointLimit requires `limit/upper` attribute on edges");
                auto upper_view = upper_attr->view();

                auto strength_attr = sc->edges().find<Float>("limit/strength");
                UIPC_ASSERT(strength_attr, "AffineBodyRevoluteJointLimit requires `limit/strength` attribute on edges");
                auto strength_view = strength_attr->view();

                for(SizeT i = 0; i < sc->edges().size(); ++i)
                {
                    UIPC_ASSERT(lower_view[i] <= upper_view[i],
                                "AffineBodyRevoluteJointLimit: requires `limit/lower <= limit/upper` on edge {}, but got lower={} upper={}",
                                i,
                                lower_view[i],
                                upper_view[i]);
                    h_lowers.push_back(lower_view[i]);
                    h_uppers.push_back(upper_view[i]);
                    h_strengths.push_back(strength_view[i]);
                }
            });

        // Sanity check: limit constitution reuses body_ids/l_basis/r_basis/
        // init_angles/current_angles from AffineBodyRevoluteJoint via the
        // shared index `I`. Both must therefore enumerate exactly the same
        // (geo, edge) sequence.
        UIPC_ASSERT(h_lowers.size() == revolute_joint->h_body_ids.size(),
                    "AffineBodyRevoluteJointLimit: joint count {} must equal "
                    "AffineBodyRevoluteJoint joint count {} (index alignment required)",
                    h_lowers.size(),
                    revolute_joint->h_body_ids.size());

        lowers.copy_from(h_lowers);
        uppers.copy_from(h_uppers);
        strengths.copy_from(h_strengths);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(lowers.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;
        namespace ERJ = sym::affine_body_revolute_joint_limit;
        auto k        = affine_body_revolute_joint_limit_compute_energy_kernel;
        int  n        = (int)lowers.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                revolute_joint->body_ids.cview(),
                revolute_joint->l_basis.cview(),
                revolute_joint->r_basis.cview(),
                revolute_joint->init_angles.cview(),
                revolute_joint->current_angles.cview(),
                lowers.cview(),
                uppers.cview(),
                strengths.cview(),
                info.qs(),
                info.q_prevs(),
                info.energies(),
                n);
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(2 * lowers.size());
        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * lowers.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;
        namespace ERJ      = sym::affine_body_revolute_joint_limit;
        auto gradient_only = info.gradient_only();

        auto k = affine_body_revolute_joint_limit_compute_gradient_hessian_kernel;
        int n = (int)lowers.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                revolute_joint->body_ids.cview(),
                revolute_joint->l_basis.cview(),
                revolute_joint->r_basis.cview(),
                revolute_joint->init_angles.cview(),
                revolute_joint->current_angles.cview(),
                lowers.cview(),
                uppers.cview(),
                strengths.cview(),
                info.qs(),
                info.q_prevs(),
                info.gradients(),
                info.hessians(),
                gradient_only,
                n);
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointLimit);

}  // namespace uipc::backend::cuda