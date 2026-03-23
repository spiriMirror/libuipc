#include <affine_body/inter_affine_body_constitution.h>
#include <affine_body/constitutions/affine_body_spherical_joint_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/enumerate.h>

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

                    // Ps[e[1]] is the anchor in world space (body1's attachment point,
                    // set as RT * r_local_pos in apply_to). Express it in each body's local frame.
                    Vector3 anchor = Ps[e[1]];

                    Vector6 rest_c;
                    rest_c.segment<3>(0) = LT.inverse() * anchor;
                    rest_c.segment<3>(3) = RT.inverse() * anchor;
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
