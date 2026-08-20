#include <affine_body/affine_body_constitution.h>
#include <affine_body/constitutions/ortho_potential_function.h>
#include <utils/make_spd.h>


namespace uipc::backend::cuda
{
namespace
{
    namespace AOP = sym::abd_ortho_potential;

    __global__ void ortho_potential_compute_energy_kernel(
        cuda_tool::BufferView<Float>     shape_energies,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::CBufferView<Float>    kappas,
        cuda_tool::CBufferView<Float>    volumes,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& q      = qs(i);
        auto& volume = volumes(i);
        auto  kappa  = kappas(i);
        Float Vdt2   = volume * dt * dt;

        Float E;
        AOP::E(E, kappa, q);

        shape_energies(i) = E * Vdt2;
    }

    __global__ void ortho_potential_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector12>    qs,
        cuda_tool::CBufferView<Float>       volumes,
        cuda_tool::BufferView<Vector12>     gradients,
        cuda_tool::BufferView<Matrix12x12>  body_hessian,
        cuda_tool::CBufferView<Float>       kappas,
        Float                               dt,
        bool                                gradient_only,
        int                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Matrix12x12 H = Matrix12x12::Zero();
        Vector12    G = Vector12::Zero();

        const auto& q      = qs(i);
        Float       kappa  = kappas(i);
        const auto& volume = volumes(i);

        Float Vdt2 = volume * dt * dt;

        Vector9 G9;
        AOP::dEdq(G9, kappa, q);
        G.segment<9>(3) = G9 * Vdt2;
        gradients(i)    = G;

        if(gradient_only)
            return;

        Matrix9x9 H9x9;
        AOP::ddEddq(H9x9, kappa, q);
        make_spd(H9x9);

        H.block<9, 9>(3, 3) = H9x9 * Vdt2;
        body_hessian(i)     = H;
    }
}  // namespace

class OrthoPotential final : public AffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 1ull;

    using AffineBodyConstitution::AffineBodyConstitution;

    vector<Float> h_kappas;

    cuda_tool::DeviceBuffer<Float> kappas;

    virtual void do_build(AffineBodyConstitution::BuildInfo& info) override {}

    U64 get_uid() const override { return ConstitutionUID; }

    void do_init(AffineBodyDynamics::FilteredInfo& info) override
    {
        using ForEachInfo = AffineBodyDynamics::ForEachInfo;

        // find out constitution coefficients
        h_kappas.resize(info.body_count());
        auto geo_slots = world().scene().geometries();

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.instances().find<Float>("kappa")->view(); },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto bodyI      = I.global_index();
                h_kappas[bodyI] = kappa;
            });

        auto async_copy = []<typename T>(span<T> src, cuda_tool::DeviceBuffer<T>& dst)
        {
            cuda_tool::BufferLaunch().resize<T>(dst, src.size());
            cuda_tool::BufferLaunch().copy<T>(dst.view(), src.data());
        };

        async_copy(span{h_kappas}, kappas);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;

        auto body_count = info.qs().size();

        namespace AOP = sym::abd_ortho_potential;

        auto k = ortho_potential_compute_energy_kernel;
        int  n = (int)body_count;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.energies(), info.qs(), kappas.cview(), info.volumes(), info.dt(), n);
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;
        auto N             = info.qs().size();
        auto gradient_only = info.gradient_only();

        namespace AOP = sym::abd_ortho_potential;

        auto k = ortho_potential_compute_gradient_hessian_kernel;
        int  n = (int)N;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.qs(),
                info.volumes(),
                info.gradients(),
                info.hessians(),
                kappas.cview(),
                info.dt(),
                gradient_only,
                n);
    }
};

REGISTER_SIM_SYSTEM(OrthoPotential);
}  // namespace uipc::backend::cuda
