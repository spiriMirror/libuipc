#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
class NeoHookeanShell2D final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 11;
    static constexpr SizeT StencilSize     = 3;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using Codim2DConstitution::Codim2DConstitution;

    vector<Float> h_lambdas;
    vector<Float> h_mus;

    muda::DeviceBuffer<Float>     lambdas;
    muda::DeviceBuffer<Float>     mus;
    muda::DeviceBuffer<Matrix2x2> inv_B_matrices;

    SimSystemSlot<FiniteElementMethod> fem;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override
    {
        fem = require<FiniteElementMethod>();
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_mus.resize(N);
        h_lambdas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto lambda = sc.triangles().find<Float>("lambda");
                auto mu     = sc.triangles().find<Float>("mu");


                return zip(lambda->view(), mu->view());
            },
            [&](const ForEachInfo& I, auto lambda_mu)
            {
                auto vI = I.global_index();

                auto&& [lambda, mu] = lambda_mu;

                h_lambdas[vI] = lambda;
                h_mus[vI]     = mu;
            });

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        auto& cinfo       = info.constitution_info();
        auto  prim_offset = cinfo.primitive_offset;
        auto  prim_count  = cinfo.primitive_count;

        auto prims  = fem->codim_2ds().subview(prim_offset, prim_count);
        auto x_bars = fem->x_bars();

        inv_B_matrices.resize(N);

        // Precompute inverse of rest shape matrix for each triangle
        using namespace muda;
        namespace NH = sym::neo_hookean_shell_2d;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [prims  = prims.viewer().name("prims"),
                    x_bars = x_bars.viewer().name("x_bars"),
                    inv_B_mats = inv_B_matrices.viewer().name("inv_B_mats")] __device__(int I)
                   {
                       Vector9  X_bar;
                       Vector3i idx = prims(I);
                       for(int i = 0; i < 3; ++i)
                           X_bar.segment<3>(3 * i) = x_bars(idx(i));
                       Matrix2x2 B;
                       NH::A(B, X_bar);

                       inv_B_mats(I) = muda::eigen::inverse(B);
                   });
    }

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(mus.size());
        info.gradient_count(mus.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(mus.size() * HalfHessianSize);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace NH = sym::neo_hookean_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [lambdas    = lambdas.cviewer().name("lambdas"),
                    mus        = mus.cviewer().name("mus"),
                    rest_areas = info.rest_areas().viewer().name("rest_area"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    x_bars   = info.x_bars().viewer().name("x_bars"),
                    IBs      = inv_B_matrices.cviewer().name("IBs"),
                    dt       = info.dt()] __device__(int I)
                   {
                       Vector9          X;
                       Vector3i         idx = indices(I);
                       const Matrix2x2& IB  = IBs(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Float lambda = lambdas(I);
                       Float mu     = mus(I);

                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));
                       Float E;
                       NH::E(E, lambda, mu, X, IB);

                       // thickness is one-sided so we multiply by 2
                       Float Vdt2  = rest_area * 2 * thickness * dt * dt;
                       energies(I) = E * Vdt2;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace NH = sym::neo_hookean_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [lambdas = lambdas.cviewer().name("lambdas"),
                    mus     = mus.cviewer().name("mus"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    IBs     = inv_B_matrices.cviewer().name("IBs"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    G3s        = info.gradients().viewer().name("gradients"),
                    H3x3s      = info.hessians().viewer().name("hessians"),
                    rest_areas = info.rest_areas().viewer().name("volumes"),
                    dt         = info.dt(),
                    half_hessian_size = HalfHessianSize,
                    gradient_only = info.gradient_only()] __device__(int I) mutable
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Matrix2x2 IB = IBs(I);

                       Float lambda    = lambdas(I);
                       Float mu        = mus(I);
                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       // thickness is one-sided so we multiply by 2
                       Float Vdt2 = rest_area * 2 * thickness * dt * dt;

                       Vector9 G;
                       NH::dEdX(G, lambda, mu, X, IB);
                       G *= Vdt2;
                       DoubletVectorAssembler DVA{G3s};
                       DVA.segment<StencilSize>(I * StencilSize).write(idx, G);

                       if(gradient_only)
                           return;

                       Matrix9x9 H;
                       NH::ddEddX(H, lambda, mu, X, IB);
                       make_spd(H);
                       H *= Vdt2;

                       TripletMatrixAssembler TMA{H3x3s};
                       TMA.half_block<StencilSize>(I * half_hessian_size).write(idx, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(NeoHookeanShell2D);
}  // namespace uipc::backend::cuda
