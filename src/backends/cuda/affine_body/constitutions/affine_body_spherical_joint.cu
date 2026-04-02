#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_spherical_joint_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
class AffineBodySphericalJoint final : public InterAffineBodyConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 26;
    static constexpr SizeT HalfHessianSize = 2 * (2 + 1) / 2;
    using InterAffineBodyConstitution::InterAffineBodyConstitution;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    muda::DeviceBuffer<Vector2i> body_ids;
    muda::DeviceBuffer<Vector6> rest_cs;  // anchor in each body's local frame [ci_bar | cj_bar]
    muda::DeviceBuffer<Float> strength_ratios;

    vector<Vector2i> h_body_ids;
    vector<Vector6>  h_rest_cs;
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

                    Vector6 rest_c;
                    if(use_local_positions)
                    {
                        rest_c.segment<3>(0) = pos0_attr->view()[i];
                        rest_c.segment<3>(3) = pos1_attr->view()[i];
                    }
                    else
                    {
                        Vector3 anchor = Ps[i];
                        rest_c.segment<3>(0) = LT.inverse() * anchor;
                        rest_c.segment<3>(3) = RT.inverse() * anchor;
                    }
                    rest_c_list.push_back(rest_c);
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
        move_to(h_strength_ratios, strength_ratio_list);

        body_ids.copy_from(h_body_ids);
        rest_cs.copy_from(h_rest_cs);
        strength_ratios.copy_from(h_strength_ratios);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(body_ids.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace SJ = sym::affine_body_spherical_joint;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_cs  = rest_cs.cviewer().name("rest_cs"),
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

                       Vector3 Ft_val;
                       SJ::Ft<Float>(Ft_val, rest_c.segment<3>(0), qi, rest_c.segment<3>(3), qj);
                       Float Et_val;
                       SJ::Et<Float>(Et_val, kappa, Ft_val);

                       Es(I) = Et_val;
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
        namespace SJ       = sym::affine_body_spherical_joint;
        auto gradient_only = info.gradient_only();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_ids.size(),
                   [body_ids = body_ids.cviewer().name("body_ids"),
                    rest_cs  = rest_cs.cviewer().name("rest_cs"),
                    strength_ratio = strength_ratios.cviewer().name("strength_ratio"),
                    body_masses = info.body_masses().cviewer().name("body_masses"),
                    qs      = info.qs().viewer().name("qs"),
                    G12s    = info.gradients().viewer().name("G12s"),
                    H12x12s = info.hessians().viewer().name("H12x12s"),
                    gradient_only] __device__(int I)
                   {
                       Vector2i bids  = body_ids(I);
                       Float    kappa = strength_ratio(I)
                                     * (body_masses(bids(0)).mass()
                                        + body_masses(bids(1)).mass());

                       Vector12 qi = qs(bids(0));
                       Vector12 qj = qs(bids(1));

                       auto&   rest_c = rest_cs(I);
                       Vector3 ci_bar = rest_c.segment<3>(0);
                       Vector3 cj_bar = rest_c.segment<3>(3);

                       Vector3 Ft_val;
                       SJ::Ft<Float>(Ft_val, ci_bar, qi, cj_bar, qj);

                       Vector3 dEdFt_val;
                       SJ::dEdFt<Float>(dEdFt_val, kappa, Ft_val);

                       Vector24 G;
                       SJ::JtT_Gt<Float>(G, dEdFt_val, ci_bar, cj_bar);

                       DoubletVectorAssembler DVA{G12s};
                       Vector2i               indices = {bids(0), bids(1)};
                       DVA.segment<2>(2 * I).write(indices, G);

                       if(!gradient_only)
                       {
                           Matrix3x3 ddEdFt_val;
                           SJ::ddEdFt<Float>(ddEdFt_val, kappa, Ft_val);
                           make_spd(ddEdFt_val);

                           Matrix24x24 H;
                           SJ::JtT_Ht_Jt<Float>(H, ddEdFt_val, ci_bar, cj_bar);

                           TripletMatrixAssembler TMA{H12x12s};
                           TMA.half_block<2>(HalfHessianSize * I).write(indices, H);
                       }
                   });
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};
REGISTER_SIM_SYSTEM(AffineBodySphericalJoint);
}  // namespace uipc::backend::cuda
