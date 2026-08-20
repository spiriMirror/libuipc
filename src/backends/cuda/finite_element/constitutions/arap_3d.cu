#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/arap_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace ARAP = sym::arap_3d;

    constexpr SizeT StencilSize     = 4;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void ARAP3D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>     kappas,
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

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        Float E;

        ARAP::E(E, kappas(I) * dt * dt, volumes(I), F);
        energies(I) = E;
    }

    __global__ void ARAP3D_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>            kappas,
        cuda_tool::CBufferView<Vector4i>         indices,
        cuda_tool::CBufferView<Vector3>          xs,
        cuda_tool::CBufferView<Matrix3x3>        Dm_invs,
        cuda_tool::DoubletVectorView<Float, 3>   G3s,
        cuda_tool::TripletMatrixView<Float, 3>   H3x3s,
        cuda_tool::CBufferView<Float>            volumes,
        Float                                    dt,
        bool                                     gradient_only,
        int                                      n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector4i&  tet    = indices(I);
        const Matrix3x3& Dm_inv = Dm_invs(I);

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        auto kt2 = kappas(I) * dt * dt;
        auto v   = volumes(I);

        Vector9 dEdF;
        ARAP::dEdF(dEdF, kt2, v, F);

        Matrix9x12 dFdx = fem::dFdx(Dm_inv);
        Vector12   G12  = dFdx.transpose() * dEdF;

        DoubletVectorAssembler DVA{G3s};
        DVA.segment<StencilSize>(I * StencilSize).write(tet, G12);

        if(gradient_only)
            return;

        Matrix9x9 ddEddF;
        ARAP::ddEddF(ddEddF, kt2, v, F);
        make_spd(ddEddF);
        Matrix12x12 H12x12 = dFdx.transpose() * ddEddF * dFdx;
        TripletMatrixAssembler TMA{H3x3s};
        TMA.half_block<StencilSize>(I * HalfHessianSize).write(tet, H12x12);
    }
}  // namespace

class ARAP3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 9;
    static constexpr SizeT StencilSize     = 4;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float> h_kappas;

    cuda_tool::DeviceBuffer<Float> kappas;

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
                auto kappa = sc.tetrahedra().find<Float>("kappa");
                UIPC_ASSERT(kappa, "Can't find attribute `kappa` on tetrahedra, why can it happen?");
                return kappa->view();
            },
            [&](const ForEachInfo& I, Float kappa)
            { h_kappas[I.global_index()] = kappa; });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = ARAP3D_do_compute_energy_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                kappas.cview(),
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
        auto k = ARAP3D_do_compute_gradient_hessian_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                kappas.cview(),
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

REGISTER_SIM_SYSTEM(ARAP3D);
}  // namespace uipc::backend::cuda
