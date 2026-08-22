#include <finite_element/fem_time_integrator.h>
#include <time_integrator/bdf1_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FEMBDF1Integrator_do_predict_dof_kernel(
        cuda_tool::CBufferView<IndexT>  is_fixed,
        cuda_tool::CBufferView<IndexT>  is_dynamic,
        cuda_tool::BufferView<Vector3>  x_prevs,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> vs,
        cuda_tool::BufferView<Vector3>  x_tildes,
        cuda_tool::CBufferView<Vector3> gravities,
        cuda_tool::CBufferView<Vector3> external_force_accs,
        Float                           dt,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // record previous position
        Vector3& x_prev = x_prevs(i);
        x_prev          = xs(i);

        const Vector3& v = vs(i);

        // 0) fixed: x_tilde = x_prev
        Vector3 x_tilde = x_prev;

        if(!is_fixed(i))
        {
            const Vector3& g         = gravities(i);
            const Vector3& f_ext_acc = external_force_accs(i);

            // 1) static problem: x_tilde = x_prev + (g + f_ext_acc) * dt * dt
            x_tilde += (g + f_ext_acc) * dt * dt;

            // 2) dynamic problem: x_tilde = x_prev + v * dt + (g + f_ext_acc) * dt * dt
            if(is_dynamic(i))
            {
                x_tilde += v * dt;
            }
        }

        x_tildes(i) = x_tilde;
    }

    __global__ void FEMBDF1Integrator_do_update_state_kernel(
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::BufferView<Vector3>  vs,
        cuda_tool::CBufferView<Vector3> x_prevs,
        Float                           dt,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector3&       v      = vs(i);
        const Vector3& x_prev = x_prevs(i);
        const Vector3& x      = xs(i);

        v = (x - x_prev) * (1.0 / dt);
    }
}  // namespace

class FEMBDF1Integrator final : public FEMTimeIntegrator
{
  public:
    using FEMTimeIntegrator::FEMTimeIntegrator;

    void do_build(BuildInfo& info) override
    {
        // require the BDF1 flag
        require<BDF1Flag>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_predict_dof(PredictDofInfo& info) override
    {
        auto k = FEMBDF1Integrator_do_predict_dof_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.is_dynamic().cview(),
                info.x_prevs(),
                info.xs().cview(),
                info.vs().cview(),
                info.x_tildes(),
                info.gravities().cview(),
                info.external_force_accs().cview(),
                info.dt(),
                n);
        }
    }

    virtual void do_update_state(UpdateVelocityInfo& info) override
    {
        auto k = FEMBDF1Integrator_do_update_state_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.xs().cview(), info.vs(), info.x_prevs().cview(), info.dt(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(FEMBDF1Integrator);
}  // namespace uipc::backend::cuda
