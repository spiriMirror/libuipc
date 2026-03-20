#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_fixed_joint_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
class AffineBodyFixedJoint final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 25;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6> rest_cs;  // midpoint in body-local frame [ci_bar | cj_bar]
    muda::DeviceBuffer<Vector6> rest_ts;  // t_bar in body space
    muda::DeviceBuffer<Vector6> rest_ns;  // n_bar in body space
    muda::DeviceBuffer<Vector6> rest_bs;  // b_bar in body space
    muda::DeviceBuffer<Float>   strength_ratios;

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
                auto sc = geo.as<geometry::SimplicialComplex>();

                auto geo_ids_view = sc->edges().find<Vector2i>("geo_ids")->view();
                auto inst_ids_view = sc->edges().find<Vector2i>("inst_ids")->view();
                auto strength_ratio_view =
                    sc->edges().find<Float>("strength_ratio")->view();

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();
                for(auto&& [i, e] : enumerate(Es))
                {
                    Vector2i geo_id  = geo_ids_view[i];
                    Vector2i inst_id = inst_ids_view[i];

                    body_ids_list.push_back({info.body_id(geo_id(0), inst_id(0)),
                                             info.body_id(geo_id(1), inst_id(1))});

                    Transform LT{
                        info.body_geo(geo_slots, geo_id(0))->transforms().view()[inst_id(0)]};
                    Transform RT{
                        info.body_geo(geo_slots, geo_id(1))->transforms().view()[inst_id(1)]};

                    Vector3 mid_point = (Ps[e[0]] + Ps[e[1]]) / 2.0;

                    // rest_cs: midpoint in each body's local frame
                    Vector6 rest_c;
                    rest_c.segment<3>(0) = LT.inverse() * mid_point;
                    rest_c.segment<3>(3) = RT.inverse() * mid_point;
                    rest_c_list.push_back(rest_c);

                    // t, n, b: coordinate axes in body space
                    Matrix3x3 LR = LT.rotation();
                    Matrix3x3 RR = RT.rotation();

                    Vector6 rest_t;
                    rest_t.segment<3>(0) = LR.inverse() * Vector3(1, 0, 0);
                    rest_t.segment<3>(3) = RR.inverse() * Vector3(1, 0, 0);
                    rest_t_list.push_back(rest_t);

                    Vector6 rest_n;
                    rest_n.segment<3>(0) = LR.inverse() * Vector3(0, 1, 0);
                    rest_n.segment<3>(3) = RR.inverse() * Vector3(0, 1, 0);
                    rest_n_list.push_back(rest_n);

                    Vector6 rest_b;
                    rest_b.segment<3>(0) = LR.inverse() * Vector3(0, 0, 1);
                    rest_b.segment<3>(3) = RR.inverse() * Vector3(0, 0, 1);
                    rest_b_list.push_back(rest_b);
                }

                std::ranges::copy(strength_ratio_view,
                                  std::back_inserter(strength_ratio_list));
            });

        auto move_to = [](auto& dst, auto& src)
        {
            dst.resize(src.size());
            std::ranges::move(src, dst.begin());
        };

        move_to(h_body_ids, body_ids_list);
        move_to(h_rest_cs, rest_c_list);
        move_to(h_rest_ts, rest_t_list);
        move_to(h_rest_ns, rest_n_list);
        move_to(h_rest_bs, rest_b_list);
        move_to(h_strength_ratios, strength_ratio_list);

        body_ids.copy_from(h_body_ids);
        rest_cs.copy_from(h_rest_cs);
        rest_ts.copy_from(h_rest_ts);
        rest_ns.copy_from(h_rest_ns);
        rest_bs.copy_from(h_rest_bs);
        strength_ratios.copy_from(h_strength_ratios);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace FJ = sym::affine_body_fixed_joint;
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
                       Vector2i bids  = body_ids(I);
                       Float    kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 qi = qs(bids(0));
                       Vector12 qj = qs(bids(1));

                       auto& rest_c = rest_cs(I);
                       auto& rest_t = rest_ts(I);
                       auto& rest_n = rest_ns(I);
                       auto& rest_b = rest_bs(I);

                       Vector9 Fr_val;
                       FJ::Fr<Float>(Fr_val,
                                     rest_t.segment<3>(0),
                                     rest_n.segment<3>(0),
                                     rest_b.segment<3>(0),
                                     qi,
                                     rest_t.segment<3>(3),
                                     rest_n.segment<3>(3),
                                     rest_b.segment<3>(3),
                                     qj);
                       Float Er_val;
                       FJ::Er(Er_val, kappa, Fr_val);

                       Vector3 Ft_val;
                       FJ::Ft<Float>(Ft_val, rest_c.segment<3>(0), qi, rest_c.segment<3>(3), qj);
                       Float Et_val;
                       FJ::Et(Et_val, kappa, Ft_val);

                       Es(I) = Er_val + Et_val;
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
        namespace FJ       = sym::affine_body_fixed_joint;
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
                       Vector2i bids  = body_ids(I);
                       Float    kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 qi = qs(bids(0));
                       Vector12 qj = qs(bids(1));

                       auto& rest_c = rest_cs(I);
                       auto& rest_t = rest_ts(I);
                       auto& rest_n = rest_ns(I);
                       auto& rest_b = rest_bs(I);

                       // Rotation gradient
                       Vector9 Fr_val;
                       FJ::Fr<Float>(Fr_val,
                                     rest_t.segment<3>(0),
                                     rest_n.segment<3>(0),
                                     rest_b.segment<3>(0),
                                     qi,
                                     rest_t.segment<3>(3),
                                     rest_n.segment<3>(3),
                                     rest_b.segment<3>(3),
                                     qj);

                       Vector9 dEdFr_val;
                       FJ::dEdFr(dEdFr_val, kappa, Fr_val);

                       Vector24 JrT_Gr_val;
                       FJ::JrT_Gr<Float>(JrT_Gr_val,
                                         dEdFr_val,
                                         rest_t.segment<3>(0),
                                         rest_n.segment<3>(0),
                                         rest_b.segment<3>(0),
                                         qi,
                                         rest_t.segment<3>(3),
                                         rest_n.segment<3>(3),
                                         rest_b.segment<3>(3),
                                         qj);

                       // Translation gradient
                       Vector3 Ft_val;
                       FJ::Ft<Float>(Ft_val, rest_c.segment<3>(0), qi, rest_c.segment<3>(3), qj);

                       Vector3 dEdFt_val;
                       FJ::dEdFt(dEdFt_val, kappa, Ft_val);

                       Vector24 JtT_Gt_val;
                       FJ::JtT_Gt<Float>(JtT_Gt_val,
                                         dEdFt_val,
                                         rest_c.segment<3>(0),
                                         rest_c.segment<3>(3));

                       Vector24               G = JrT_Gr_val + JtT_Gt_val;
                       DoubletVectorAssembler DVA{G12s};
                       Vector2i               indices = {bids(0), bids(1)};
                       DVA.segment<2>(2 * I).write(indices, G);

                       if(!gradient_only)
                       {
                           Matrix9x9 ddEdFr_val;
                           FJ::ddEdFr(ddEdFr_val, kappa, Fr_val);
                           make_spd(ddEdFr_val);

                           Matrix24x24 JrT_Hr_Jr_val;
                           FJ::JrT_Hr_Jr<Float>(JrT_Hr_Jr_val,
                                                ddEdFr_val,
                                                rest_t.segment<3>(0),
                                                rest_n.segment<3>(0),
                                                rest_b.segment<3>(0),
                                                qi,
                                                rest_t.segment<3>(3),
                                                rest_n.segment<3>(3),
                                                rest_b.segment<3>(3),
                                                qj);

                           Matrix3x3 ddEdFt_val;
                           FJ::ddEdFt(ddEdFt_val, kappa, Ft_val);
                           make_spd(ddEdFt_val);

                           Matrix24x24 JtT_Ht_Jt_val;
                           FJ::JtT_Ht_Jt<Float>(JtT_Ht_Jt_val,
                                                ddEdFt_val,
                                                rest_c.segment<3>(0),
                                                rest_c.segment<3>(3));

                           Matrix24x24 H = JrT_Hr_Jr_val + JtT_Ht_Jt_val;
                           TripletMatrixAssembler TMA{H12x12s};
                           TMA.half_block<2>(HalfHessianSize * I).write(indices, H);
                       }
                   });
    };

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodyFixedJoint);
}  // namespace uipc::backend::cuda
