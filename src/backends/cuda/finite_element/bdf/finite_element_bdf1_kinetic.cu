#include <time_integrator/bdf1_flag.h>
#include <finite_element/finite_element_kinetic.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementBDF1Kinetic_do_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT>  is_fixed,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> x_tildes,
        cuda_tool::CBufferView<Float>   masses,
        cuda_tool::BufferView<Float>    Ks,
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
            const Vector3& x       = xs(i);
            const Vector3& x_tilde = x_tildes(i);
            Float          M       = masses(i);
            Vector3        dx      = x - x_tilde;
            K                      = 0.5 * M * dx.dot(dx);
        }
    }

    __global__ void FiniteElementBDF1Kinetic_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<IndexT>            is_fixed,
        cuda_tool::CBufferView<Vector3>           xs,
        cuda_tool::CBufferView<Vector3>           x_tildes,
        cuda_tool::CBufferView<Float>             masses,
        cuda_tool::DoubletVectorView<Float, 3>    G3s,
        cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s,
        bool                                      gradient_only,
        int                                       n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& m       = masses(i);
        auto& x       = xs(i);
        auto& x_tilde = x_tildes(i);

        Vector3 G;

        if(is_fixed(i))  // fixed
        {
            G = Vector3::Zero();
        }
        else
        {
            G = m * (x - x_tilde);
        }

        G3s(i).write(i, G);

        if(gradient_only)
            return;

        Matrix3x3 H = masses(i) * Matrix3x3::Identity();
        H3x3s(i).write(i, i, H);
    }
}  // namespace

class FiniteElementBDF1Kinetic final : public FiniteElementKinetic
{
  public:
    using FiniteElementKinetic::FiniteElementKinetic;

    virtual void do_build(BuildInfo& info) override
    {
        // require BDF1 integration flag
        require<BDF1Flag>();
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        // Compute kinetic energy
        auto k = FiniteElementBDF1Kinetic_do_compute_energy_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.xs().cview(),
                info.x_tildes(),
                info.masses().cview(),
                info.energies(),
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        auto k = FiniteElementBDF1Kinetic_do_compute_gradient_hessian_kernel;
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
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(FiniteElementBDF1Kinetic);
}  // namespace uipc::backend::cuda
