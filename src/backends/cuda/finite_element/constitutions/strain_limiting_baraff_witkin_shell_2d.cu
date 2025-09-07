#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
class StrianLimitingBaraffWitkinShell2D final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 18;

    using Codim2DConstitution::Codim2DConstitution;

    vector<Float> h_kappas;
    vector<Float> h_lambdas;
    vector<Float> h_strainRates;

    muda::DeviceBuffer<Float> kappas;
    muda::DeviceBuffer<Float> lambdas;
    muda::DeviceBuffer<Float> strainRates;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);
        h_lambdas.resize(N);
        h_strainRates.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto mu     = sc.triangles().find<Float>("mu");
                auto lambda = sc.triangles().find<Float>("lambda");

                return zip(mu->view(), lambda->view());
            },
            [&](const ForEachInfo& I, auto mu_and_lambda)
            {
                auto vI = I.global_index();

                auto&& [mu, lambda] = mu_and_lambda;
                h_kappas[vI]        = mu;
                h_lambdas[vI]       = lambda;
                h_strainRates[vI]    = 100;
            });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());

        strainRates.resize(N);
        strainRates.view().copy_from(h_strainRates.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus        = kappas.cviewer().name("mus"),
                    lambdas    = lambdas.cviewer().name("lambdas"),
                    strainRates    = strainRates.cviewer().name("strainRates"),
                    rest_areas = info.rest_areas().viewer().name("rest_area"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    energies = info.energies().viewer().name("energies"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    x_bars  = info.x_bars().viewer().name("x_bars"),
                    dt      = info.dt()] __device__(int I)
                   {
                       Vector9  X;
                       Vector3i idx = indices(I);
                       for(int i = 0; i < 3; ++i)
                           X.segment<3>(3 * i) = xs(idx(i));

                       Matrix2x2 IB = BWS::Dm2x2(x_bars(idx(0)), x_bars(idx(1)), x_bars(idx(2)));
                       IB = muda::eigen::inverse(IB);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Matrix2x2 A = BWS::Dm2x2(X.segment<3>(0), X.segment<3>(3), X.segment<3>(6));

                           Float detA = A.determinant();
                       }

                       Float mu        = mus(I);
                       Float lambda    = lambdas(I);
                       Float strainRate = strainRates(I);
                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Matrix<Float, 3, 2> Ds = BWS::Ds3x2(X.segment<3>(0), X.segment<3>(3), X.segment<3>(6));
                       Matrix<Float, 3, 2> F = Ds * IB;

                       Vector2 anisotropic_a = Vector2(1, 0);
                       Vector2 anisotropic_b = Vector2(0, 1);

                       Float E = BWS::E(F, anisotropic_a, anisotropic_b, mu, lambda, strainRate);
                       energies(I) = E * rest_area * thickness * dt * dt;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.indices().size(),
                   [mus     = kappas.cviewer().name("mus"),
                    lambdas = lambdas.cviewer().name("lambdas"),
                    strainRates = strainRates.cviewer().name("strainRates"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    x_bars  = info.x_bars().viewer().name("x_bars"),
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

                       Matrix2x2 IB =
                           BWS::Dm2x2(x_bars(idx(0)), x_bars(idx(1)), x_bars(idx(2)));
                       IB = muda::eigen::inverse(IB);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Matrix2x2 A = BWS::Dm2x2(X.segment<3>(0),
                                                    X.segment<3>(3),
                                                    X.segment<3>(6));

                           Float detA = A.determinant();
                       }

                       Float mu        = mus(I);
                       Float lambda    = lambdas(I);
                       Float strainRate = strainRates(I);
                       Float rest_area = rest_areas(I);
                       Float thickness = triangle_thickness(thicknesses(idx(0)),
                                                            thicknesses(idx(1)),
                                                            thicknesses(idx(2)));

                       Matrix<Float, 3, 2> Ds =
                           BWS::Ds3x2(X.segment<3>(0), X.segment<3>(3), X.segment<3>(6));
                       Matrix<Float, 3, 2> F = Ds * IB;

                       Vector2 anisotropic_a = Vector2(1, 0);
                       Vector2 anisotropic_b = Vector2(0, 1);

                       auto dFdx = BWS::dFdX(IB);

                       Float Vdt2 = rest_area * thickness * dt * dt;

                       Matrix<Float, 3, 2> dEdF;
                       BWS::dEdF(dEdF, F, anisotropic_a, anisotropic_b, mu, lambda, strainRate);

                       auto VecdEdF = BWS::flatten(dEdF);

                       Vector9 G = dFdx.transpose() * VecdEdF;

                       G *= Vdt2;
                       DoubletVectorAssembler DVA{G3s};
                       DVA.segment<3>(I * 3).write(idx, G);

                       Matrix6x6 ddEddF;
                       BWS::ddEddF(ddEddF, F, anisotropic_a, anisotropic_b, mu, lambda, strainRate);
                       ddEddF *= Vdt2;
                       
                       Matrix9x9 H = dFdx.transpose() * ddEddF * dFdx;
                       TripletMatrixAssembler TMA{H3x3s};
                       TMA.block<3, 3>(I * 3 * 3).write(idx, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(StrianLimitingBaraffWitkinShell2D);
}  // namespace uipc::backend::cuda
