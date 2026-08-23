#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    // Stiff-GIPC SNK1 (energy, gradient and the analytically SPD-projected
    // Hessian) replaces the SymEigen-generated SNH + generic make_spd EVD
    namespace SNH = snk1;

    constexpr SizeT StencilSize     = 4;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void StableNeoHookean3D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>     mus,
        cuda_tool::CBufferView<Float>     lambdas,
        cuda_tool::BufferView<Float>      energies,
        cuda_tool::CBufferView<Vector4i>  indices,
        cuda_tool::CBufferView<Vector3>   xs,
        cuda_tool::CBufferView<Matrix3x3> Dm_invs,
        cuda_tool::CBufferView<Float>     volumes,
        Float                             dt,
        int                               n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector4i&  tet    = indices(I);
        const Matrix3x3& Dm_inv = Dm_invs(I);
        Float            mu     = mus(I);
        Float            lambda = lambdas(I);

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        auto J = F.determinant();

        //auto VecF = flatten(F);

        Float E;

        SNH::E(E, mu, lambda, F);
        E *= dt * dt * volumes(I);
        energies(I) = E;
    }

    __global__ void StableNeoHookean3D_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>          mus,
        cuda_tool::CBufferView<Float>          lambdas,
        cuda_tool::CBufferView<Vector4i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Matrix3x3>      Dm_invs,
        cuda_tool::DoubletVectorView<Float, 3> G3s,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          volumes,
        Float                                  dt,
        bool                                   gradient_only,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector4i&  tet    = indices(I);
        const Matrix3x3& Dm_inv = Dm_invs(I);
        Float            mu     = mus(I);
        Float            lambda = lambdas(I);

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        auto J = F.determinant();

        //auto VecF = flatten(F);

        auto Vdt2 = volumes(I) * dt * dt;

        Matrix3x3 dEdF;
        SNH::dEdF(dEdF, mu, lambda, F);
        auto VecdEdF = flatten(dEdF);
        VecdEdF *= Vdt2;

        Matrix9x12 dFdx = fem::dFdx(Dm_inv);
        Vector12   G    = dFdx.transpose() * VecdEdF;

        DoubletVectorAssembler DVA{G3s};
        DVA.segment<StencilSize>(I * StencilSize).write(tet, G);

        if(gradient_only)
            return;

        // analytically SPD-projected 9x9 energy Hessian (Stiff SNK1);
        // scaling by the positive Vdt2 commutes with the projection
        Matrix9x9 ddEddF;
        SNH::ddEddF_spd(ddEddF, mu, lambda, F);
        ddEddF *= Vdt2;
        Matrix12x12            H = dFdx.transpose() * ddEddF * dFdx;
        TripletMatrixAssembler TMA{H3x3s};
        TMA.half_block<StencilSize>(I * HalfHessianSize).write(tet, H);
    }
}  // namespace

class StableNeoHookean3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 10;
    static constexpr SizeT StencilSize     = 4;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float> h_mus;
    vector<Float> h_lambdas;

    cuda_tool::DeviceBuffer<Float> mus;
    cuda_tool::DeviceBuffer<Float> lambdas;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(mus.size());
        info.gradient_count(mus.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(mus.size() * HalfHessianSize);
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
                auto mu     = sc.tetrahedra().find<Float>("mu");
                auto lambda = sc.tetrahedra().find<Float>("lambda");

                return zip(mu->view(), lambda->view());
            },
            [&](const ForEachInfo& I, auto mu_and_lambda)
            {
                auto&& [mu, lambda] = mu_and_lambda;

                auto vI = I.global_index();

                h_mus[vI]     = mu;
                h_lambdas[vI] = lambda;
            });

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = StableNeoHookean3D_do_compute_energy_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mus.cview(),
                lambdas.cview(),
                info.energies(),
                info.indices(),
                info.xs(),
                info.Dm_invs(),
                info.rest_volumes(),
                info.dt(),
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        auto k = StableNeoHookean3D_do_compute_gradient_hessian_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mus.cview(),
                lambdas.cview(),
                info.indices(),
                info.xs(),
                info.Dm_invs(),
                info.gradients(),
                info.hessians(),
                info.rest_volumes(),
                info.dt(),
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(StableNeoHookean3D);
}  // namespace uipc::backend::cuda
