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
}  // namespace uipc::backend::cuda