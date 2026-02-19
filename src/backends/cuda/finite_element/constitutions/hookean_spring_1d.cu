#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_1d_constitution_diff_parm_reporter.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembler.h>
#include <numbers>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
class HookeanSpring1D final : public Codim1DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 12ull;
    static constexpr SizeT StencilSize     = 2;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using Codim1DConstitution::Codim1DConstitution;

    vector<Float>             h_kappas;
    muda::DeviceBuffer<Float> kappas;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(kappas.size());
        info.gradient_count(kappas.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(kappas.size() * HalfHessianSize);
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto kappa = sc.edges().find<Float>("kappa");
                return kappa->view();
            },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto vI = I.global_index();
                // retrieve material parameters
                h_kappas[vI] = kappa;
            });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace NS = sym::hookean_spring_1d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [kappas = kappas.cviewer().name("kappas"),
                    rest_lengths = info.rest_lengths().viewer().name("rest_lengths"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    x_bars   = info.x_bars().viewer().name("x_bars"),
                    dt       = info.dt(),
                    Pi       = std::numbers::pi] __device__(int I)
                   {
                       Vector6  X;
                       Vector2i idx = indices(I);
                       for(int i = 0; i < 2; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Float L0 = rest_lengths(I);
                       Float r =
                           edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
                       Float kappa = kappas(I);

                       Float Vdt2 = L0 * r * r * Pi * dt * dt;

                       Float E;
                       NS::E(E, kappa, X, L0);
                       energies(I) = E * Vdt2;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace NS       = sym::hookean_spring_1d;
        auto gradient_only = info.gradient_only();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [G3s    = info.gradients().viewer().name("gradients"),
                    H3x3s  = info.hessians().viewer().name("hessians"),
                    kappas = kappas.cviewer().name("kappas"),
                    rest_lengths = info.rest_lengths().viewer().name("rest_lengths"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    x_bars  = info.x_bars().viewer().name("x_bars"),
                    dt      = info.dt(),
                    Pi      = std::numbers::pi,
                    gradient_only] __device__(int I) mutable
                   {
                       Vector6  X;
                       Vector2i idx = indices(I);
                       for(int i = 0; i < 2; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Float L0 = rest_lengths(I);
                       Float r =
                           edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
                       Float kappa = kappas(I);

                       Float Vdt2 = L0 * r * r * Pi * dt * dt;

                       Vector6 G;
                       NS::dEdX(G, kappa, X, L0);
                       G *= Vdt2;
                       DoubletVectorAssembler VA{G3s};
                       VA.segment<StencilSize>(I * StencilSize).write(idx, G);

                       if(!gradient_only)
                       {
                           Matrix6x6 H;
                           NS::ddEddX(H, kappa, X, L0);
                           H *= Vdt2;
                           make_spd(H);
                           TripletMatrixAssembler MA{H3x3s};
                           MA.half_block<StencilSize>(I * HalfHessianSize).write(idx, H);
                       }
                   });
    }
};

REGISTER_SIM_SYSTEM(HookeanSpring1D);
}  // namespace uipc::backend::cuda
