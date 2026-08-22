#include <affine_body/abd_time_integrator.h>
#include <time_integrator/bdf1_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_bdf1_integrator_predict_dof_kernel(
        cuda_tool::CBufferView<IndexT>   is_fixed,
        cuda_tool::CBufferView<IndexT>   is_dynamic,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::BufferView<Vector12>  q_prevs,
        cuda_tool::CBufferView<Vector12> q_vs,
        cuda_tool::BufferView<Vector12>  q_tildes,
        cuda_tool::CBufferView<Vector12> affine_gravity,
        cuda_tool::CBufferView<Vector12> external_force_accs,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // record previous q
        auto& q_prev = q_prevs(i);
        q_prev       = qs(i);

        auto& q_v       = q_vs(i);
        auto& g         = affine_gravity(i);
        auto& f_ext_acc = external_force_accs(i);

        // 0) fixed: q_tilde = q_prev;
        Vector12 q_tilde = q_prev;

        if(!is_fixed(i))
        {
            // 1) static problem: q_tilde = q_prev + (g + f_ext_acc) * dt * dt;
            q_tilde += (g + f_ext_acc) * dt * dt;

            // 2) dynamic problem q_tilde = q_prev + q_v * dt + (g + f_ext_acc) * dt * dt;
            if(is_dynamic(i))
            {
                q_tilde += q_v * dt;
            }
        }

        q_tildes(i) = q_tilde;
    }

    __global__ void abd_bdf1_integrator_update_state_kernel(
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::BufferView<Vector12>  q_vs,
        cuda_tool::CBufferView<Vector12> q_prevs,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& q_v    = q_vs(i);
        auto& q_prev = q_prevs(i);

        const auto& q = qs(i);

        q_v = (q - q_prev) * (1.0 / dt);
    }
}  // namespace

class ABDBDF1Integrator final : public ABDTimeIntegrator
{
  public:
    using ABDTimeIntegrator::ABDTimeIntegrator;

    void do_build(BuildInfo& info) override
    {
        // require the BDF1 flag
        require<BDF1Flag>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_predict_dof(PredictDofInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = abd_bdf1_integrator_predict_dof_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.is_dynamic().cview(),
                info.qs().cview(),
                info.q_prevs(),
                info.q_vs().cview(),
                info.q_tildes(),
                info.gravities().cview(),
                info.external_force_accs().cview(),
                info.dt(),
                n);
        }
    }

    virtual void do_update_state(UpdateVelocityInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = abd_bdf1_integrator_update_state_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.qs().cview(), info.q_vs(), info.q_prevs().cview(), info.dt(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(ABDBDF1Integrator);
}  // namespace uipc::backend::cuda