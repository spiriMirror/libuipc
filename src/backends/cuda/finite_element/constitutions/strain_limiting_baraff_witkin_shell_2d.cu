#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <cuda_tool/cuda_tool.h>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace BWS = sym::strainlimiting_baraff_witkin_shell_2d;
    namespace eigen = cuda_tool::eigen;

    constexpr SizeT StencilSize     = 3;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void StrainLimitingBaraffWitkinShell2D_do_init_kernel(
        cuda_tool::CBufferView<Vector3i>  indices,
        cuda_tool::CBufferView<Vector3>   x_bars,
        cuda_tool::BufferView<Matrix2x2>  inv_Bs,
        int                               n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector3i  tri = indices(I);
        Matrix2x2 Dm =
            BWS::Dm2x2(x_bars(tri(0)), x_bars(tri(1)), x_bars(tri(2)));
        inv_Bs(I) = eigen::inverse(Dm);
    }

    __global__ void StrainLimitingBaraffWitkinShell2D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>     mus,
        cuda_tool::CBufferView<Float>     lambdas,
        cuda_tool::CBufferView<Float>     strain_rates,
        cuda_tool::CBufferView<Float>     rest_areas,
        cuda_tool::CBufferView<Float>     thicknesses,
        cuda_tool::BufferView<Float>      energies,
        cuda_tool::CBufferView<Vector3i>  indices,
        cuda_tool::CBufferView<Vector3>   xs,
        cuda_tool::CBufferView<Matrix2x2> IBs,
        Float                             dt,
        int                               n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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
    }

    __global__ void StrainLimitingBaraffWitkinShell2D_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>            mus,
        cuda_tool::CBufferView<Float>            lambdas,
        cuda_tool::CBufferView<Float>            strainRates,
        cuda_tool::CBufferView<Vector3i>         indices,
        cuda_tool::CBufferView<Vector3>          xs,
        cuda_tool::CBufferView<Float>            thicknesses,
        cuda_tool::DoubletVectorView<Float, 3>   G3s,
        cuda_tool::TripletMatrixView<Float, 3>   H3x3s,
        cuda_tool::CBufferView<Float>            rest_areas,
        Float                                    dt,
        cuda_tool::CBufferView<Matrix2x2>        IBs,
        SizeT                                    half_hessian_size,
        bool                                     gradient_only,
        int                                      n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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
    }
}  // namespace

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

    cuda_tool::DeviceBuffer<Float>     mus;
    cuda_tool::DeviceBuffer<Float>     lambdas;
    cuda_tool::DeviceBuffer<Float>     strain_rates;
    cuda_tool::DeviceBuffer<Matrix2x2> inv_B_matrices;

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

        // Precompute inverse of rest shape matrix for each triangle
        auto k = StrainLimitingBaraffWitkinShell2D_do_init_kernel;
        int  n = (int)prims.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                prims.cview(), x_bars.cview(), inv_B_matrices.view(), n);
        }
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
        auto k = StrainLimitingBaraffWitkinShell2D_do_compute_energy_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mus.cview(),
                lambdas.cview(),
                strain_rates.cview(),
                info.rest_areas(),
                info.thicknesses(),
                info.energies(),
                info.indices(),
                info.xs(),
                inv_B_matrices.cview(),
                info.dt(),
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        auto k = StrainLimitingBaraffWitkinShell2D_do_compute_gradient_hessian_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mus.cview(),
                lambdas.cview(),
                strain_rates.cview(),
                info.indices(),
                info.xs(),
                info.thicknesses(),
                info.gradients(),
                info.hessians(),
                info.rest_areas(),
                info.dt(),
                inv_B_matrices.cview(),
                HalfHessianSize,
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(StrainLimitingBaraffWitkinShell2D);
}  // namespace uipc::backend::cuda
