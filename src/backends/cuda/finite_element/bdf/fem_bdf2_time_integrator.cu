#include <finite_element/fem_time_integrator.h>
#include <time_integrator/bdf2_flag.h>
#include <finite_element/bdf/fem_bdf2_state.h>
#include <finite_element/bdf/bdf2_function.h>

namespace uipc::backend::cuda
{
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
        using namespace muda;
        namespace BDF2 = sym::fem_bdf2;

        if(state->x_n_1s().size() != info.xs().size())
        {
            state->resize(info.xs().size());
            // Bootstrap history from current x_n and v_n.
            state->x_n_1s().copy_from(info.x_prevs());
            state->v_n_1s().copy_from(info.vs());
        }

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [is_fixed   = info.is_fixed().cviewer().name("is_fixed"),
                    is_dynamic = info.is_dynamic().cviewer().name("is_dynamic"),
                    xs         = info.xs().cviewer().name("xs"),
                    x_ns       = info.x_prevs().viewer().name("x_ns"),
                    x_n_1s     = state->x_n_1s().cviewer().name("x_n_1s"),
                    v_ns       = info.vs().viewer().name("v_ns"),
                    v_n_1s     = state->v_n_1s().cviewer().name("v_n_1s"),
                    x_tildes   = info.x_tildes().viewer().name("x_tilde"),
                    gravity    = info.gravities().cviewer().name("gravity"),
                    external_force_accs = info.external_force_accs().cviewer().name("external_force_accs"),
                    dt         = info.dt()] __device__(int i) mutable
                   {
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
                   });
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace BDF2 = sym::fem_bdf2;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [xs     = info.xs().cviewer().name("xs"),
                    x_ns   = info.x_prevs().cviewer().name("x_n"),
                    x_n_1s = state->x_n_1s().viewer().name("x_n_1s"),
                    v_ns   = info.vs().viewer().name("v_ns"),
                    v_n_1s = state->v_n_1s().viewer().name("v_n_1s"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
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
                   });
    }
};

REGISTER_SIM_SYSTEM(FEMBDF2Integrator);
}  // namespace uipc::backend::cuda
