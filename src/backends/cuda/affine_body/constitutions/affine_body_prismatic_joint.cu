#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>
#include <time_integrator/time_integrator.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/offset_count_collection.h>
#include <utils/make_spd.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
class AffineBodyPrismaticJoint final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 20;

    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    vector<Vector2i> h_body_ids;
    // [    body0   |   body1    ]
    vector<Vector6> h_rest_positions_c;
    vector<Vector6> h_rest_positions_d;
    vector<Vector6> h_rest_positions_n;
    vector<Vector6> h_rest_positions_b;
    vector<Float>   h_strength_ratios;

    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6>  rest_positions_c;
    muda::DeviceBuffer<Vector6>  rest_positions_d;
    muda::DeviceBuffer<Vector6>  rest_positions_n;
    muda::DeviceBuffer<Vector6>  rest_positions_b;
    muda::DeviceBuffer<Float>    strength_ratios;

    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_init(FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        list<Vector2i> body_ids_list;
        list<Vector6>  rest_positions_c_list;
        list<Vector6>  rest_positions_d_list;
        list<Vector6>  rest_positions_n_list;
        list<Vector6>  rest_positions_b_list;
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

                    Vector2i geo_id = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)), info.body_id(geo_id(1), inst_id(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                    auto right_sc = info.body_geo(geo_slots, geo_id(1));

                    UIPC_ASSERT(inst_id(0) >= 0 && inst_id(0) < static_cast<IndexT>(left_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Left instance ID {} is out of range [0, {})",
                                inst_id(0),
                                left_sc->instances().size());
                    UIPC_ASSERT(inst_id(1) >= 0 && inst_id(1) < static_cast<IndexT>(right_sc->instances().size()),
                                "AffineBodyPrismaticJoint: Right instance ID {} is out of range [0, {})",
                                inst_id(1),
                                right_sc->instances().size());

                    Transform LT{left_sc->transforms().view()[inst_id(0)]};
                    Transform RT{right_sc->transforms().view()[inst_id(1)]};

                    Vector3 tangent   = (P1 - P0).normalized();
                    Vector3 normal    = Normal(tangent);
                    Vector3 bitangent = normal.cross(tangent).normalized();

                    Vector6 rest_position_c;
                    rest_position_c.segment<3>(0) = LT.inverse() * P0;  // C_p_bar
                    rest_position_c.segment<3>(3) = RT.inverse() * P0;  // C_q_bar
                    rest_positions_c_list.push_back(rest_position_c);

                    Vector6 rest_position_d;
                    rest_position_d.segment<3>(0) = LT.inverse() * (P0 + tangent);  // D_p_bar
                    rest_position_d.segment<3>(3) = RT.inverse() * (P0 + tangent);  // D_q_bar
                    rest_positions_d_list.push_back(rest_position_d);

                    Matrix3x3 LT_rotation = LT.rotation();
                    Matrix3x3 RT_rotation = RT.rotation();
                    Vector6   rest_position_n;
                    rest_position_n.segment<3>(0) = LT_rotation.inverse() * normal;  // N_p_bar
                    rest_position_n.segment<3>(3) = RT_rotation.inverse() * normal;  // N_q_bar
                    rest_positions_n_list.push_back(rest_position_n);

                    Vector6 rest_position_b;
                    rest_position_b.segment<3>(0) = LT_rotation.inverse() * bitangent;  // B_p_bar
                    rest_position_b.segment<3>(3) = RT_rotation.inverse() * bitangent;  // B_q_bar
                    rest_positions_b_list.push_back(rest_position_b);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
            });

        h_body_ids.resize(body_ids_list.size());
        std::ranges::move(body_ids_list, h_body_ids.begin());

        h_rest_positions_c.resize(rest_positions_c_list.size());
        std::ranges::move(rest_positions_c_list, h_rest_positions_c.begin());

        h_rest_positions_d.resize(rest_positions_d_list.size());
        std::ranges::move(rest_positions_d_list, h_rest_positions_d.begin());

        h_rest_positions_n.resize(rest_positions_n_list.size());
        std::ranges::move(rest_positions_n_list, h_rest_positions_n.begin());

        h_rest_positions_b.resize(rest_positions_b_list.size());
        std::ranges::move(rest_positions_b_list, h_rest_positions_b.begin());

        h_strength_ratios.resize(strength_ratio_list.size());
        std::ranges::move(strength_ratio_list, h_strength_ratios.begin());

        body_ids.copy_from(h_body_ids);
        rest_positions_c.copy_from(h_rest_positions_c);
        rest_positions_d.copy_from(h_rest_positions_d);
        rest_positions_n.copy_from(h_rest_positions_n);
        rest_positions_b.copy_from(h_rest_positions_b);
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
                    rest_positions_c = rest_positions_c.cviewer().name("rest_positions_c"),
                    rest_positions_d = rest_positions_d.cviewer().name("rest_positions_d"),
                    rest_positions_n = rest_positions_n.cviewer().name("rest_positions_n"),
                    rest_positions_b = rest_positions_b.cviewer().name("rest_positions_b"),
                    strength_ratio = strength_ratios.cviewer().name("strength_ratio"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    qs = info.qs().viewer().name("qs"),
                    Es = info.energies().viewer().name("Es")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Float kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 p = qs(bids(0));
                       Vector12 q = qs(bids(1));

                       const Vector6& C_bar = rest_positions_c(I);
                       const Vector6& D_bar = rest_positions_d(I);
                       const Vector6& N_bar = rest_positions_n(I);
                       const Vector6& B_bar = rest_positions_b(I);

                       ABDJacobi JC[2] = {ABDJacobi{C_bar.segment<3>(0)},
                                          ABDJacobi{C_bar.segment<3>(3)}};
                       ABDJacobi JD[2] = {ABDJacobi{D_bar.segment<3>(0)},
                                          ABDJacobi{D_bar.segment<3>(3)}};
                       ABDJacobi JN[2] = {ABDJacobi{N_bar.segment<3>(0)},
                                          ABDJacobi{N_bar.segment<3>(3)}};
                       ABDJacobi JB[2] = {ABDJacobi{B_bar.segment<3>(0)},
                                          ABDJacobi{B_bar.segment<3>(3)}};


                       Vector3 Cp = JC[0].point_x(p);
                       Vector3 Cq = JC[1].point_x(q);
                       Vector3 Dp = JD[0].point_x(p);
                       Vector3 Dq = JD[1].point_x(q);

                       Vector3 Np = JN[0].vec_x(p);
                       Vector3 Nq = JN[1].vec_x(q);

                       Vector3 Bp = JB[0].vec_x(p);
                       Vector3 Bq = JB[1].vec_x(q);

                       // E0 = 1/2 * kappa * ||(C_q-C_p) x (D_p-C_p) ||^2
                       Float E0 = 0.5 * kappa * (Cq - Cp).cross(Dp - Cp).squaredNorm();
                       // E1 = 1/2 * kappa * ||(C_p-C_q) x (D_q-C_q) ||^2
                       Float E1 = 0.5 * kappa * (Cp - Cq).cross(Dq - Cq).squaredNorm();

                       // E2 = 1/2 * kappa *  ||N_p-N_q|^2
                       Float E2 = 0.5 * kappa * (Np - Nq).squaredNorm();

                       // E3 = 1/2 * kappa * ||B_p-B_q|^2
                       Float E3 = 0.5 * kappa * (Bp - Bq).squaredNorm();

                       Es(I) = E0 + E1 + E2 + E3;
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_segment_count(2 * body_ids.size());  // each joint has 2 * Vector12 gradients
        info.hessian_block_count(4 * body_ids.size());  // each joint has 2 * 2 * Matrix12x12 hessians
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace PJ = sym::affine_body_prismatic_joint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                body_ids.size(),
                [body_ids = body_ids.cviewer().name("body_ids"),
                 rest_positions_c = rest_positions_c.cviewer().name("rest_positions_c"),
                 rest_positions_d = rest_positions_d.cviewer().name("rest_positions_d"),
                 rest_positions_n = rest_positions_n.cviewer().name("rest_positions_n"),
                 rest_positions_b = rest_positions_b.cviewer().name("rest_positions_b"),
                 strength_ratio = strength_ratios.cviewer().name("strength_ratio"),
                 body_masses = info.body_masses().cviewer().name("body_masses"),
                 qs          = info.qs().cviewer().name("qs"),
                 G12s        = info.gradients().viewer().name("G12s"),
                 H12x12s = info.hessians().viewer().name("H12x12s")] __device__(int I)
                {
                    Vector2i bids = body_ids(I);

                    Float kappa =
                        strength_ratio(I)
                        * (body_masses(bids(0)).mass() + body_masses(bids(1)).mass());

                    Vector12 p = qs(bids(0));
                    Vector12 q = qs(bids(1));

                    const Vector6& C_bar = rest_positions_c(I);
                    const Vector6& D_bar = rest_positions_d(I);
                    const Vector6& N_bar = rest_positions_n(I);
                    const Vector6& B_bar = rest_positions_b(I);


                    ABDJacobi JC[2] = {ABDJacobi{C_bar.segment<3>(0)},
                                       ABDJacobi{C_bar.segment<3>(3)}};
                    ABDJacobi JD[2] = {ABDJacobi{D_bar.segment<3>(0)},
                                       ABDJacobi{D_bar.segment<3>(3)}};
                    ABDJacobi JN[2] = {ABDJacobi{N_bar.segment<3>(0)},
                                       ABDJacobi{N_bar.segment<3>(3)}};
                    ABDJacobi JB[2] = {ABDJacobi{B_bar.segment<3>(0)},
                                       ABDJacobi{B_bar.segment<3>(3)}};

                    Vector3 Cp = JC[0].point_x(p);
                    Vector3 Cq = JC[1].point_x(q);
                    Vector3 Dp = JD[0].point_x(p);
                    Vector3 Dq = JD[1].point_x(q);

                    Vector3 Np = JN[0].vec_x(p);
                    Vector3 Nq = JN[1].vec_x(q);

                    Vector3 Bp = JB[0].vec_x(p);
                    Vector3 Bq = JB[1].vec_x(q);

                    Vector9 Q0;
                    Q0.segment<3>(0) = Cp;
                    Q0.segment<3>(3) = Dp;
                    Q0.segment<3>(6) = Cq;

                    Vector9 Q1;
                    Q1.segment<3>(0) = Cq;
                    Q1.segment<3>(3) = Dq;
                    Q1.segment<3>(6) = Cp;

                    Vector6 N;
                    N.segment<3>(0) = Np;
                    N.segment<3>(3) = Nq;

                    Vector6 B;
                    B.segment<3>(0) = Bp;
                    B.segment<3>(3) = Bq;

                    //Fill Body Gradient
                    //dE0 / d[p, q] = dE0 / dQ0 * dQ0 / d[p, q]  = JQ0 ^ T* dE0 / dQ0 Vector<Float, 24> G24_0;
                    Vector<Float, 24> G24_0;
                    PJ::GradE(
                        G24_0, kappa, Q0, JC[0].x_bar(), JD[0].x_bar(), JC[1].x_bar());
                    // dE1 / d[q,p] = dE0 / dQ1 * dQ1 / d[q,p] = JQ1^T *  dE0 / dQ1
                    Vector<Float, 24> G24_1;
                    PJ::GradE(
                        G24_1, kappa, Q1, JC[1].x_bar(), JD[1].x_bar(), JC[0].x_bar());
                    // dE2 / d[q,p] = JN^T * dE2 / dN
                    Vector<Float, 24> G24_N;
                    PJ::GradX(G24_N, kappa, N, JN[0].x_bar(), JN[1].x_bar());
                    // dE3 / d[q,p] = JB^T * dE2 / dB
                    Vector<Float, 24> G24_B;
                    PJ::GradX(G24_B, kappa, B, JB[0].x_bar(), JB[1].x_bar());
                    {
                        // E0
                        Vector12 G_i0 = G24_0.segment<12>(0);
                        // E1
                        Vector12 G_i1 = G24_1.segment<12>(12);
                        // E2 (normal)
                        Vector12 G_i2 = G24_N.segment<12>(0);
                        // E3 (bitagent)
                        Vector12 G_i3 = G24_B.segment<12>(0);
                        G12s(2 * I + 0).write(bids(0), G_i0 + G_i1 + G_i2 + G_i3);
                    }
                    {
                        // E0
                        Vector12 G_j0 = G24_0.segment<12>(12);
                        // E1
                        Vector12 G_j1 = G24_1.segment<12>(0);
                        // E2
                        Vector12 G_j2 = G24_N.segment<12>(12);
                        // E3
                        Vector12 G_j3 = G24_B.segment<12>(12);
                        G12s(2 * I + 1).write(bids(1), G_j0 + G_j1 + G_j2 + G_j3);
                    }
                    // Fill Body Hessian
                    // E0
                    Matrix9x9 ddEddQ0;
                    PJ::ddEddQ(ddEddQ0, kappa, Q0);
                    make_spd(ddEddQ0);
                    Matrix<Float, 24, 24> H24x24_0;
                    PJ::HessE(
                        H24x24_0, kappa, JC[0].x_bar(), JD[0].x_bar(), JC[1].x_bar(), ddEddQ0);
                    // E1
                    Matrix9x9 ddEddQ1;
                    PJ::ddEddQ(ddEddQ1, kappa, Q1);
                    make_spd(ddEddQ1);
                    Matrix<Float, 24, 24> H24x24_1;
                    PJ::HessE(
                        H24x24_1, kappa, JC[1].x_bar(), JD[1].x_bar(), JC[0].x_bar(), ddEddQ1);
                    // E2
                    Matrix6x6 ddEddN;
                    PJ::ddEddX(ddEddN, kappa, N);
                    make_spd(ddEddN);
                    Matrix<Float, 24, 24> H24x24_N;
                    PJ::HessX(H24x24_N, kappa, JN[0].x_bar(), JN[1].x_bar(), ddEddN);
                    // E3
                    Matrix6x6 ddEddB;
                    PJ::ddEddX(ddEddB, kappa, B);
                    make_spd(ddEddB);
                    Matrix<Float, 24, 24> H24x24_B;
                    PJ::HessX(H24x24_B, kappa, JB[0].x_bar(), JB[1].x_bar(), ddEddB);
                    {
                        Matrix12x12 H_ii_0 = H24x24_0.block(0, 0, 12, 12);
                        Matrix12x12 H_ii_1 = H24x24_1.block(12, 12, 12, 12);
                        Matrix12x12 H_ii_2 = H24x24_N.block(0, 0, 12, 12);
                        Matrix12x12 H_ii_3 = H24x24_B.block(0, 0, 12, 12);
                        H12x12s(4 * I + 0).write(bids(0), bids(0), H_ii_0 + H_ii_1 + H_ii_2 + H_ii_3);
                    }
                    {
                        // ij
                        Matrix12x12 H_ij_0 = H24x24_0.block(0, 12, 12, 12);
                        Matrix12x12 H_ij_1 = H24x24_1.block(12, 0, 12, 12);
                        Matrix12x12 H_ij_2 = H24x24_N.block(0, 12, 12, 12);
                        Matrix12x12 H_ij_3 = H24x24_B.block(0, 12, 12, 12);
                        // ji
                        Matrix12x12 H_ji_0 = H24x24_0.block(12, 0, 12, 12);
                        Matrix12x12 H_ji_1 = H24x24_1.block(0, 12, 12, 12);
                        Matrix12x12 H_ji_2 = H24x24_N.block(12, 0, 12, 12);
                        Matrix12x12 H_ji_3 = H24x24_B.block(12, 0, 12, 12);
                        H12x12s(4 * I + 1).write(bids(0), bids(1), H_ij_0 + H_ij_1 + H_ij_2 + H_ij_3);
                        H12x12s(4 * I + 2).write(bids(1), bids(0), H_ji_0 + H_ji_1 + H_ji_2 + H_ji_3);
                    }
                    {
                        Matrix12x12 H_jj_0 = H24x24_0.block(12, 12, 12, 12);
                        Matrix12x12 H_jj_1 = H24x24_1.block(0, 0, 12, 12);
                        Matrix12x12 H_jj_2 = H24x24_N.block(12, 12, 12, 12);
                        Matrix12x12 H_jj_3 = H24x24_B.block(12, 12, 12, 12);
                        H12x12s(4 * I + 3).write(bids(1), bids(1), H_jj_0 + H_jj_1 + H_jj_2 + H_jj_3);
                    }
                });
    };


    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJoint);
}  // namespace uipc::backend::cuda