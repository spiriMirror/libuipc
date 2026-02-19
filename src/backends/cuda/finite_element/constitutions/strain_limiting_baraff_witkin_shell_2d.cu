#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
class StrainLimitingBaraffWitkinShell2D final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 819;
    static constexpr SizeT StencilSize     = 3;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using Codim2DConstitution::Codim2DConstitution;

    vector<Float> h_mus;
    vector<Float> h_lambdas;
    vector<Float> h_strain_rates;

    muda::DeviceBuffer<Float>     mus;
    muda::DeviceBuffer<Float>     lambdas;
    muda::DeviceBuffer<Float>     strain_rates;
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
        h_strain_rates.resize(N);

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

                h_lambdas[vI]      = lambda;
                h_mus[vI]          = mu;
                h_strain_rates[vI] = 100;
            });

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());

        strain_rates.resize(N);
        strain_rates.view().copy_from(h_strain_rates.data());

        auto& cinfo       = info.constitution_info();
        auto  prim_offset = cinfo.primitive_offset;
        auto  prim_count  = cinfo.primitive_count;

        auto prims  = fem->codim_2ds().subview(prim_offset, prim_count);
        auto x_bars = fem->x_bars();

        inv_B_matrices.resize(N);

        using namespace muda;
        namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

        // Precompute inverse of rest shape matrix for each triangle
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(prims.size(),
                   [indices = prims.cviewer().name("prims"),
                    x_bars  = x_bars.cviewer().name("x_bars"),
                    inv_Bs = inv_B_matrices.viewer().name("inv_Bs")] __device__(int I)
                   {
                       Vector3i  tri = indices(I);
                       Matrix2x2 Dm =
                           BWS::Dm2x2(x_bars(tri(0)), x_bars(tri(1)), x_bars(tri(2)));
                       inv_Bs(I) = eigen::inverse(Dm);
                   });
    }

    virtual void do_report_extent(ReportExtentInfo& info)
    {
        info.energy_count(h_mus.size());
        info.gradient_count(h_mus.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(h_mus.size() * HalfHessianSize);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus          = mus.cviewer().name("mus"),
                    lambdas      = lambdas.cviewer().name("lambdas"),
                    strain_rates = strain_rates.cviewer().name("strain_rates"),
                    rest_areas   = info.rest_areas().viewer().name("rest_area"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices  = info.indices().viewer().name("indices"),
                    xs       = info.xs().viewer().name("xs"),
                    IBs      = inv_B_matrices.cviewer().name("IBs"),
                    dt       = info.dt()] __device__(int I)
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       const Matrix2x2& IB = IBs(I);

                       Float lambda      = lambdas(I);
                       Float mu          = mus(I);
                       Float strain_rate = strain_rates(I);
                       Float rest_area   = rest_areas(I);

                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Matrix<Float, 3, 2> Ds =
                           BWS::Ds3x2(X.segment<3>(0), X.segment<3>(3), X.segment<3>(6));
                       Matrix<Float, 3, 2> F = Ds * IB;

                       Vector2 anisotropic_a = Vector2(1, 0);
                       Vector2 anisotropic_b = Vector2(0, 1);

                       // thickness is onesided, so the Volume is area * thickness * 2
                       Float V = rest_area * thickness * 2;

                       Float E = BWS::E(F, anisotropic_a, anisotropic_b, lambda, mu, strain_rate);
                       energies(I) = E * V * dt * dt;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus         = mus.cviewer().name("mus"),
                    lambdas     = lambdas.cviewer().name("lambdas"),
                    strainRates = strain_rates.cviewer().name("strainRates"),
                    indices     = info.indices().viewer().name("indices"),
                    xs          = info.xs().viewer().name("xs"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    G3s        = info.gradients().viewer().name("gradients"),
                    H3x3s      = info.hessians().viewer().name("hessians"),
                    rest_areas = info.rest_areas().viewer().name("volumes"),
                    dt         = info.dt(),
                    IBs        = inv_B_matrices.cviewer().name("IBs"),
                    half_hessian_size = HalfHessianSize,
                    gradient_only = info.gradient_only()] __device__(int I) mutable
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       const Matrix2x2& IB = IBs(I);

                       Float lambda      = lambdas(I);
                       Float mu          = mus(I);
                       Float strain_rate = strainRates(I);
                       Float rest_area   = rest_areas(I);

                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Matrix<Float, 3, 2> Ds =
                           BWS::Ds3x2(X.segment<3>(0), X.segment<3>(3), X.segment<3>(6));
                       Matrix<Float, 3, 2> F = Ds * IB;

                       Vector2 anisotropic_a = Vector2(1, 0);
                       Vector2 anisotropic_b = Vector2(0, 1);

                       auto dFdx = BWS::dFdX(IB);

                       Float V = 2 * rest_area * thickness;

                       Float Vdt2 = V * dt * dt;

                       Matrix<Float, 3, 2> dEdF;
                       BWS::dEdF(dEdF, F, anisotropic_a, anisotropic_b, lambda, mu, strain_rate);

                       auto VecdEdF = BWS::flatten(dEdF);

                       Vector9 G = dFdx.transpose() * VecdEdF;

                       G *= Vdt2;
                       DoubletVectorAssembler DVA{G3s};
                       DVA.segment<StencilSize>(I * StencilSize).write(idx, G);

                       if(gradient_only)
                           return;

                       Matrix6x6 ddEddF;
                       BWS::ddEddF(ddEddF, F, anisotropic_a, anisotropic_b, lambda, mu, strain_rate);

                       ddEddF *= Vdt2;

                       Matrix9x9 H = dFdx.transpose() * ddEddF * dFdx;

                       TripletMatrixAssembler TMA{H3x3s};
                       TMA.half_block<StencilSize>(I * half_hessian_size).write(idx, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(StrainLimitingBaraffWitkinShell2D);
}  // namespace uipc::backend::cuda
