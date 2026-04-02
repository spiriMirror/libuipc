#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_fixed_joint_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>

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

                auto l_geo_id = sc->vertices().find<IndexT>("l_geo_id");
                auto l_geo_id_view = l_geo_id->view();
                auto r_geo_id = sc->vertices().find<IndexT>("r_geo_id");
                auto r_geo_id_view = r_geo_id->view();
                auto l_inst_id = sc->vertices().find<IndexT>("l_inst_id");
                auto l_inst_id_view = l_inst_id->view();
                auto r_inst_id = sc->vertices().find<IndexT>("r_inst_id");
                auto r_inst_id_view = r_inst_id->view();
                auto strength_ratio_view =
                    sc->vertices().find<Float>("strength_ratio")->view();

                auto pos0_attr = sc->vertices().find<Vector3>("l_position");
                auto pos1_attr = sc->vertices().find<Vector3>("r_position");
                const bool use_local_positions = pos0_attr && pos1_attr;

                auto Ps = sc->positions().view();
                const SizeT n_joints = sc->vertices().size();
                for(SizeT i = 0; i < n_joints; ++i)
                {
                    IndexT l_gid = l_geo_id_view[i];
                    IndexT r_gid = r_geo_id_view[i];
                    IndexT l_iid = l_inst_id_view[i];
                    IndexT r_iid = r_inst_id_view[i];

                    body_ids_list.push_back({info.body_id(l_gid, l_iid),
                                             info.body_id(r_gid, r_iid)});

                    Transform LT{
                        info.body_geo(geo_slots, l_gid)->transforms().view()[l_iid]};
                    Transform RT{
                        info.body_geo(geo_slots, r_gid)->transforms().view()[r_iid]};

                    // rest_cs: matching attachment points in each body's local frame
                    Vector6 rest_c;
                    if(use_local_positions)
                    {
                        rest_c.segment<3>(0) = pos0_attr->view()[i];
                        rest_c.segment<3>(3) = pos1_attr->view()[i];
                    }
                    else
                    {
                        Vector3 mid_point = Ps[i];
                        rest_c.segment<3>(0) = LT.inverse() * mid_point;
                        rest_c.segment<3>(3) = RT.inverse() * mid_point;
                    }
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
