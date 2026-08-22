#include <affine_body/abd_time_integrator.h>
#include <time_integrator/bdf2_flag.h>
#include <affine_body/bdf/abd_bdf2_state.h>
#include <affine_body/bdf/bdf2_function.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace BDF2 = sym::abd_bdf2;

    __global__ void abd_bdf2_integrator_predict_dof_kernel(
        cuda_tool::CBufferView<IndexT>   is_fixed,
        cuda_tool::CBufferView<IndexT>   is_dynamic,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::BufferView<Vector12>  q_ns,
        cuda_tool::CBufferView<Vector12> q_n_1s,
        cuda_tool::CBufferView<Vector12> q_v_ns,
        cuda_tool::CBufferView<Vector12> q_v_n_1s,
        cuda_tool::BufferView<Vector12>  q_tildes,
        cuda_tool::CBufferView<Vector12> gravity,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // q_n is tracked in q_prevs for current step.
        q_ns(i)          = qs(i);
        auto&    q_n     = q_ns(i);
        Vector12 q_n_1   = q_n_1s(i);
        Vector12 q_v_n   = q_v_ns(i);
        Vector12 q_v_n_1 = q_v_n_1s(i);
        Vector12 g       = gravity(i);

        if(is_fixed(i))
        {
            // Remove all influence.
            q_n_1 = q_n;
            q_v_n.setZero();
            q_v_n_1.setZero();
            g.setZero();
        }
        else if(!is_dynamic(i))
        {
            // Remove velocity influence in static solve.
            q_n_1 = q_n;
            q_v_n.setZero();
            q_v_n_1.setZero();
        }

        Vector12 q_tilde;
        BDF2::compute_q_tilde(q_tilde, q_n, q_n_1, q_v_n, q_v_n_1, g, dt);
        q_tildes(i) = q_tilde;
    }

    __global__ void abd_bdf2_integrator_update_state_kernel(
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::CBufferView<Vector12> q_ns,
        cuda_tool::BufferView<Vector12>  q_n_1s,
        cuda_tool::BufferView<Vector12>  q_v_ns,
        cuda_tool::BufferView<Vector12>  q_v_n_1s,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector12    qv;
        const auto& q     = qs(i);
        const auto& q_n   = q_ns(i);
        auto&       q_n_1 = q_n_1s(i);
        BDF2::compute_qv(qv, q, q_n, q_n_1, dt);

        // Update velocity history.
        q_v_n_1s(i) = q_v_ns(i);
        q_v_ns(i)   = qv;

        // Update position history.
        q_n_1s(i) = q_n;
    }
}  // namespace

class ABDBDF2Integrator final : public ABDTimeIntegrator
{
  public:
    using ABDTimeIntegrator::ABDTimeIntegrator;

    SimSystemSlot<ABDBDF2State> state;

    void do_build(BuildInfo& info) override
    {
        // require the BDF2 flag
        require<BDF2Flag>();

        state = require<ABDBDF2State>();
    }

    void do_init(InitInfo& info) override {}

    void do_predict_dof(PredictDofInfo& info) override
    {
        if(state->q_n_1s().size() != info.qs().size())
        {
            state->resize(info.qs().size());
            // Bootstrap history from current q_n and v_n.
            state->q_n_1s().copy_from(info.q_prevs());
            state->q_v_n_1s().copy_from(info.q_vs());
        }

        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = abd_bdf2_integrator_predict_dof_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.is_dynamic().cview(),
                info.qs().cview(),
                info.q_prevs(),
                state->q_n_1s().cview(),
                info.q_vs(),
                state->q_v_n_1s().cview(),
                info.q_tildes(),
                info.gravities().cview(),
                info.dt(),
                n);
        }
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        int n = (int)info.qs().size();
        if(n > 0)
        {
            auto k = abd_bdf2_integrator_update_state_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.qs().cview(),
                info.q_prevs().cview(),
                state->q_n_1s(),
                info.q_vs(),
                state->q_v_n_1s(),
                info.dt(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(ABDBDF2Integrator);
}  // namespace uipc::backend::cuda
