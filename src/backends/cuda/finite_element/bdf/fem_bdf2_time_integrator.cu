#include <finite_element/fem_time_integrator.h>
#include <time_integrator/bdf2_flag.h>
#include <finite_element/bdf/fem_bdf2_state.h>
#include <finite_element/bdf/bdf2_function.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace BDF2 = sym::fem_bdf2;

    __global__ void FEMBDF2Integrator_do_predict_dof_kernel(
        cuda_tool::CBufferView<IndexT>  is_fixed,
        cuda_tool::CBufferView<IndexT>  is_dynamic,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::BufferView<Vector3>  x_ns,
        cuda_tool::CBufferView<Vector3> x_n_1s,
        cuda_tool::CBufferView<Vector3> v_ns,
        cuda_tool::CBufferView<Vector3> v_n_1s,
        cuda_tool::BufferView<Vector3>  x_tildes,
        cuda_tool::CBufferView<Vector3> gravity,
        cuda_tool::CBufferView<Vector3> external_force_accs,
        Float                           dt,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // x_n is tracked in x_prevs for current step.
        x_ns(i) = xs(i);
        auto&   x_n   = x_ns(i);
        Vector3 x_n_1 = x_n_1s(i);
        Vector3 v_n   = v_ns(i);
        Vector3 v_n_1 = v_n_1s(i);
        Vector3 g     = gravity(i) + external_force_accs(i);

        if(is_fixed(i))
        {
            // Remove all influence.
            x_n_1 = x_n;
            v_n.setZero();
            v_n_1.setZero();
            g.setZero();
        }
        else if(!is_dynamic(i))
        {
            // Remove velocity influence in static solve.
            x_n_1 = x_n;
            v_n.setZero();
            v_n_1.setZero();
        }

        Vector3 x_tilde;
        BDF2::compute_x_tilde(x_tilde, x_n, x_n_1, v_n, v_n_1, g, dt);
        x_tildes(i) = x_tilde;
    }

    __global__ void FEMBDF2Integrator_do_update_state_kernel(
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> x_ns,
        cuda_tool::BufferView<Vector3>  x_n_1s,
        cuda_tool::BufferView<Vector3>  v_ns,
        cuda_tool::BufferView<Vector3>  v_n_1s,
        Float                           dt,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector3 v;
        const auto& x     = xs(i);
        const auto& x_n   = x_ns(i);
        auto&   x_n_1 = x_n_1s(i);
        BDF2::compute_v(v, x, x_n, x_n_1, dt);

        // Update velocity history.
        v_n_1s(i) = v_ns(i);
        v_ns(i)   = v;

        // Update position history.
        x_n_1s(i) = x_n;
    }
}  // namespace

class FEMBDF2Integrator final : public FEMTimeIntegrator
{
  public:
    using FEMTimeIntegrator::FEMTimeIntegrator;

    SimSystemSlot<FEMBDF2State> state;

    void do_build(BuildInfo& info) override
    {
        // require the BDF2 flag
        require<BDF2Flag>();

        state = require<FEMBDF2State>();
    }

    void do_init(InitInfo& info) override {}

    void do_predict_dof(PredictDofInfo& info) override
    {
        if(state->x_n_1s().size() != info.xs().size())
        {
            state->resize(info.xs().size());
            // Bootstrap history from current x_n and v_n.
            state->x_n_1s().copy_from(info.x_prevs());
            state->v_n_1s().copy_from(info.vs());
        }

        auto k = FEMBDF2Integrator_do_predict_dof_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.is_dynamic().cview(),
                info.xs().cview(),
                info.x_prevs(),
                state->x_n_1s().cview(),
                info.vs(),
                state->v_n_1s().cview(),
                info.x_tildes(),
                info.gravities().cview(),
                info.external_force_accs().cview(),
                info.dt(),
                n);
        }
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        auto k = FEMBDF2Integrator_do_update_state_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.xs().cview(),
                info.x_prevs().cview(),
                state->x_n_1s(),
                info.vs(),
                state->v_n_1s(),
                info.dt(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(FEMBDF2Integrator);
}  // namespace uipc::backend::cuda
