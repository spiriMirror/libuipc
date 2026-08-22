#include <affine_body/affine_body_kinetic.h>
#include <time_integrator/bdf2_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void affine_body_bdf2_kinetic_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT>              is_fixed,
        cuda_tool::CBufferView<IndexT>              ext_kinetic,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_tildes,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> masses,
        cuda_tool::BufferView<Float>                Ks,
        Float                                       inv_beta,
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
            const auto  dq      = q - q_tilde;
            K                   = (0.5 * inv_beta) * dq.dot(M * dq);
        }
    }

    __global__ void affine_body_bdf2_kinetic_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<IndexT>              is_fixed,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_tildes,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> masses,
        cuda_tool::BufferView<Matrix12x12>          hessians,
        cuda_tool::BufferView<Vector12>             gradients,
        bool                                        gradient_only,
        Float                                       inv_beta,
        int                                         n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const auto& M       = masses(i);
        const auto& q       = qs(i);
        const auto& q_tilde = q_tildes(i);

        auto& G = gradients(i);
        G       = inv_beta * (M * (q - q_tilde));

        if(is_fixed(i))
        {
            G = Vector12::Zero();
        }

        if(gradient_only)
            return;

        hessians(i) = inv_beta * M.to_mat();
    }
}  // namespace

class AffineBodyBDF2Kinetic final : public AffineBodyKinetic
{
  public:
    static constexpr Float beta     = 4.0 / 9.0;  // BDF2 beta coefficient
    static constexpr Float inv_beta = Float{1} / beta;

    using AffineBodyKinetic::AffineBodyKinetic;

    void do_build(BuildInfo& info) override
    {
        // need BDF2 flag for BDF2 time integration
        require<BDF2Flag>();
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = affine_body_bdf2_kinetic_compute_energy_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.external_kinetic().cview(),
                info.qs().cview(),
                info.q_tildes().cview(),
                info.masses().cview(),
                info.energies(),
                inv_beta,
                n);
        }
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = affine_body_bdf2_kinetic_compute_gradient_hessian_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.qs().cview(),
                info.q_tildes().cview(),
                info.masses().cview(),
                info.hessians(),
                info.gradients(),
                info.gradient_only(),
                inv_beta,
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(AffineBodyBDF2Kinetic);
}  // namespace uipc::backend::cuda
