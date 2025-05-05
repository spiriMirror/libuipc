#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembly_utils.h>

namespace uipc::backend::cuda
{
class NeoHookeanPattern final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 66ull;

    using Codim2DConstitution::Codim2DConstitution;

    vector<Float> h_kappas;
    vector<Float> h_lambdas;
    vector<Float> h_X_rests;

    muda::DeviceBuffer<Float> kappas;
    muda::DeviceBuffer<Float> lambdas;
    muda::DeviceBuffer<Float> X_rests;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);
        h_lambdas.resize(N);
        h_X_rests.resize(N*9);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) {
                auto mu      = sc.triangles().find<Float   >("mu");
                auto lambda  = sc.triangles().find<Float   >("lambda");
                auto X_rest  = sc.triangles().find<Vector9 >("X_rest");
                return zip(mu->view(), lambda->view(), X_rest->view());
                // return zip(mu->view(), lambda->view());
            },
            [&](const ForEachInfo& I, auto triple) {
                std::size_t v = I.global_index();
                auto&& [mu, lambda, Xr] = triple;
                // auto&& [mu, lambda] = triple;
        
                h_kappas [v] = mu;
                h_lambdas[v] = lambda;
        
                for (int j = 0; j < 9; ++j)
                    h_X_rests[v * 9 + j] = Xr(j);
            });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());

        X_rests.resize(N * 9);
        X_rests.view().copy_from(h_X_rests.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace NH = sym::shell_neo_hookean_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus        = kappas.cviewer().name("mus"),
                    lambdas    = lambdas.cviewer().name("lambdas"),
                    rest_areas = info.rest_areas().viewer().name("rest_area"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    // X_bars   = info.x_bars().viewer().name("x_bars"),
                    X_bars   = X_rests.cviewer().name("X_bars"),
                    dt       = info.dt()] __device__(int I)
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Vector9 X_bar;
                       for(int i = 0; i < 9; ++i)
                           X_bar(i) = X_bars(I * 9 + i);
                    //    Vector9 X_bar;
                    //    for(int i = 0; i < 3; ++i)
                    //        X_bar.segment<3>(3 * i) = X_bars(idx(i));

                       Matrix2x2 IB;
                       NH::A(IB, X_bar);
                       IB = muda::eigen::inverse(IB);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Matrix2x2 A;
                           NH::A(A, X);
                           Float detA = A.determinant();
                       }

                       Float mu        = mus(I);
                       Float lambda    = lambdas(I);
                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Float E;
                       NH::E(E, mu, lambda, X, IB);
                       energies(I) = E * rest_area * thickness * dt * dt;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace NH = sym::shell_neo_hookean_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus     = kappas.cviewer().name("mus"),
                    lambdas = lambdas.cviewer().name("lambdas"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    // X_bars  = info.x_bars().viewer().name("x_bars"),
                    X_bars   = X_rests.cviewer().name("X_bars"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    G3s        = info.gradients().viewer().name("gradient"),
                    H3x3s      = info.hessians().viewer().name("hessian"),
                    rest_areas = info.rest_areas().viewer().name("volumes"),
                    dt         = info.dt()] __device__(int I) mutable
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Vector9 X_bar;
                       for(int i = 0; i < 9; ++i)
                           X_bar(i) = X_bars(I * 9 + i);
                    //    Vector9 X_bar;
                    //    for(int i = 0; i < 3; ++i)
                    //        X_bar.segment<3>(3 * i) = X_bars(idx(i));

                       Matrix2x2 IB;
                       NH::A(IB, X_bar);
                       IB = muda::eigen::inverse(IB);

                       Float mu        = mus(I);
                       Float lambda    = lambdas(I);
                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Float Vdt2 = rest_area * thickness * dt * dt;

                       Vector9 G;
                       NH::dEdX(G, mu, lambda, X, IB);
                       G *= Vdt2;
                       assemble<3>(G3s, I * 3, idx, G);

                       Matrix9x9 H;
                       NH::ddEddX(H, mu, lambda, X, IB);
                       H *= Vdt2;
                       make_spd(H);
                       assemble<3>(H3x3s, I * 3 * 3, idx, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(NeoHookeanPattern);
}  // namespace uipc::backend::cuda
