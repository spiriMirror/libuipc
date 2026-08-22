#include <affine_body/affine_body_kinetic.h>
#include <time_integrator/bdf1_flag.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void affine_body_bdf1_kinetic_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT>              is_fixed,
        cuda_tool::CBufferView<IndexT>              ext_kinetic,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_tildes,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> masses,
        cuda_tool::BufferView<Float>                Ks,
        int                                         n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& K = Ks(i);
        if(is_fixed(i) || ext_kinetic(i))
        {
            K = 0.0;
        }
        else
        {
            const auto& q       = qs(i);
            const auto& q_tilde = q_tildes(i);
            const auto& M       = masses(i);
            Vector12    dq      = q - q_tilde;
            K                   = 0.5 * dq.dot(M * dq);
        }
    }

    __global__ void affine_body_bdf1_kinetic_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<IndexT>              is_fixed,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_prevs,
        cuda_tool::CBufferView<Vector12>            q_tildes,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> masses,
        cuda_tool::BufferView<Matrix12x12>          hessians,
        cuda_tool::BufferView<Vector12>             gradients,
        bool                                        gradient_only,
        int                                         n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const auto& q       = qs(i);
        const auto& q_prev  = q_prevs(i);
        const auto& q_tilde = q_tildes(i);
        auto&       G       = gradients(i);
        const auto& M       = masses(i);

        G = M * (q - q_tilde);


        if(is_fixed(i))
        {
            G = Vector12::Zero();
        }

        // cout << "KG(" << i << "): " << G.transpose().eval() << "\n";

        if(gradient_only)
            return;

        hessians(i) = M.to_mat();
    }
}  // namespace

class AffineBodyBDF1Kinetic final : public AffineBodyKinetic
{
  public:
    using AffineBodyKinetic::AffineBodyKinetic;

    virtual void do_build(BuildInfo& info) override
    {
        // need BDF1 flag for BDF1 time integration
        require<BDF1Flag>();
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = affine_body_bdf1_kinetic_compute_energy_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.external_kinetic().cview(),
                info.qs().cview(),
                info.q_tildes().cview(),
                info.masses().cview(),
                info.energies(),
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = affine_body_bdf1_kinetic_compute_gradient_hessian_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.qs().cview(),
                info.q_prevs().cview(),
                info.q_tildes().cview(),
                info.masses().cview(),
                info.hessians(),
                info.gradients(),
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(AffineBodyBDF1Kinetic);
}  // namespace uipc::backend::cuda
