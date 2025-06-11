#include <affine_body/inter_affine_body_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>

namespace uipc::backend::cuda
{
static constexpr U64          ConstitutionUID = 18;
class AffineBodyRevoluteJoint final : public InterAffineBodyConstitution
{
  public:
    using InterAffineBodyConstitution::InterAffineBodyConstitution;

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
            [&](geometry::Geometry& geo)
            {
                auto uid = geo.meta().find<U64>(builtin::constitution_uid);
                U64  uid_value = uid->view()[0];
                UIPC_ASSERT(uid_value == ConstitutionUID,
                            "AffineBodyRevoluteJoint: Geometry constitution UID mismatch");

                auto sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc, "AffineBodyRevoluteJoint: Geometry must be a simplicial complex");

                auto links = sc->edges().find<Vector2i>("links");
                UIPC_ASSERT(links, "AffineBodyRevoluteJoint: Geometry must have 'links' attribute on `edges`");
                auto links_view = links->view();

                auto strength_ratio = sc->edges().find<Float>("strength_ratio");
                UIPC_ASSERT(strength_ratio, "AffineBodyRevoluteJoint: Geometry must have 'strength_ratio' attribute on `edges`");
                auto strength_ratio_view = strength_ratio->view();

                auto Es = sc->edges().topo().view();
                auto Ps = sc->positions().view();
                for(auto&& [e, link] : zip(Es, links_view))
                {
                    Vector3 P0 = Ps[e[0]];
                    Vector3 P1 = Ps[e[1]];

                    Vector2i body_ids = {info.body_id(link(0)), info.body_id(link(1))};
                    body_ids_list.push_back(body_ids);

                    auto left_sc  = info.body_geo(geo_slots, link(0));
                    auto right_sc = info.body_geo(geo_slots, link(1));
                    UIPC_ASSERT(left_sc->instances().size() == 1,
                                "AffineBodyRevoluteJoint: Left body must have exactly one instance");
                    UIPC_ASSERT(right_sc->instances().size() == 1,
                                "AffineBodyRevoluteJoint: Right body must have exactly one instance");

                    Transform LT{left_sc->transforms().view()[0]};
                    Transform RT{right_sc->transforms().view()[0]};

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
                   })
            .wait();
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_segment_count(2 * body_ids.size());  // each joint has 2 * Vector12 gradients
        info.hessian_block_count(4 * body_ids.size());  // each joint has 2 * 2 * Matrix12x12 hessians
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
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
                    qs   = info.qs().viewer().name("qs"),
                    G12s = info.gradients().viewer().name("G12s"),
                    H12x12s = info.hessians().viewer().name("H12x12s")] __device__(int I)
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
                       Float   K   = strength_ratio(I)
                                 * (body_masses(bids(0)).mass()
                                    + body_masses(bids(1)).mass());

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
                       {
                           Matrix12x12 H_ii;
                           RJ::Hess(H_ii,
                                    K,
                                    Js[0].x_bar(),
                                    Js[0].x_bar(),
                                    Js[1].x_bar(),
                                    Js[1].x_bar());
                           H12x12s(4 * I + 0).write(bids(0), bids(0), H_ii);
                       }
                       {
                           Matrix12x12 H_ij;
                           RJ::Hess(H_ij,
                                    -K,
                                    Js[0].x_bar(),
                                    Js[2].x_bar(),
                                    Js[1].x_bar(),
                                    Js[3].x_bar());
                           H12x12s(4 * I + 1).write(bids(0), bids(1), H_ij);
                           H12x12s(4 * I + 2).write(bids(1), bids(0), H_ij.transpose());
                       }
                       {
                           Matrix12x12 H_jj;
                           RJ::Hess(H_jj,
                                    K,
                                    Js[2].x_bar(),
                                    Js[2].x_bar(),
                                    Js[3].x_bar(),
                                    Js[3].x_bar());
                           H12x12s(4 * I + 3).write(bids(1), bids(1), H_jj);
                       }
                   })
            .wait();
    }

    U64 get_uid() const noexcept override { return ConstitutionUID; }
};

REGISTER_SIM_SYSTEM(AffineBodyRevoluteJoint);
}  // namespace uipc::backend::cuda