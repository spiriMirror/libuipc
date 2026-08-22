#include <finite_element/finite_element_kinetic.h>
#include <time_integrator/bdf2_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementBDF2Kinetic_do_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT>  is_fixed,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> x_tildes,
        cuda_tool::CBufferView<Float>   masses,
        cuda_tool::BufferView<Float>    Ks,
        Float                           inv_beta,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& K = Ks(i);

        if(is_fixed(i))
        {
            K = 0.0;
        }
        else
        {
            const auto& x       = xs(i);
            const auto& x_tilde = x_tildes(i);
            const auto& M       = masses(i);
            const auto  dx      = x - x_tilde;

            K = inv_beta / 2 * dx.dot(M * dx);
        }
    }

    __global__ void FiniteElementBDF2Kinetic_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<IndexT>            is_fixed,
        cuda_tool::CBufferView<Vector3>           xs,
        cuda_tool::CBufferView<Vector3>           x_tildes,
        cuda_tool::CBufferView<Float>             masses,
        cuda_tool::DoubletVectorView<Float, 3>    G3s,
        cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s,
        bool                                      gradient_only,
        Float                                     inv_beta,
        int                                       n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& m       = masses(i);
        auto& x       = xs(i);
        auto& x_tilde = x_tildes(i);

        Vector3 G;

        if(is_fixed(i))
        {
            G = Vector3::Zero();
        }
        else
        {
            G = inv_beta * m * (x - x_tilde);
        }

        G3s(i).write(i, G);

        if(gradient_only)
            return;

        const Matrix3x3 H = (inv_beta * masses(i)) * Matrix3x3::Identity();
        H3x3s(i).write(i, i, H);
    }
}  // namespace

class FiniteElementBDF2Kinetic final : public FiniteElementKinetic
{
  public:
    static constexpr Float beta     = 4.0 / 9.0;  // BDF2 beta coefficient
    static constexpr Float inv_beta = Float{1} / beta;

    using FiniteElementKinetic::FiniteElementKinetic;

    void do_build(BuildInfo& info) override
    {
        // need BDF2 flag for BDF2 time integration
        require<BDF2Flag>();
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = FiniteElementBDF2Kinetic_do_compute_energy_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.xs().cview(),
                info.x_tildes().cview(),
                info.masses().cview(),
                info.energies(),
                inv_beta,
                n);
        }
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        auto k = FiniteElementBDF2Kinetic_do_compute_gradient_hessian_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.xs().cview(),
                info.x_tildes(),
                info.masses().cview(),
                info.gradients(),
                info.hessians(),
                info.gradient_only(),
                inv_beta,
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(FiniteElementBDF2Kinetic);
}  // namespace uipc::backend::cuda
