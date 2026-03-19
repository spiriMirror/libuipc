#include <affine_body/abd_time_integrator.h>
#include <time_integrator/bdf2_flag.h>
#include <affine_body/bdf/abd_bdf2_state.h>
#include <affine_body/bdf/bdf2_function.h>

namespace uipc::backend::cuda
{
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
        using namespace muda;
        namespace BDF2 = sym::abd_bdf2;

        if(state->q_n_1s().size() != info.qs().size())
        {
            state->resize(info.qs().size());
            // Bootstrap history from current q_n and v_n.
            state->q_n_1s().copy_from(info.q_prevs());
            state->q_v_n_1s().copy_from(info.q_vs());
        }

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [is_fixed   = info.is_fixed().cviewer().name("is_fixed"),
                    is_dynamic = info.is_dynamic().cviewer().name("is_dynamic"),
                    qs         = info.qs().cviewer().name("qs"),
                    q_ns       = info.q_prevs().viewer().name("q_ns"),
                    q_n_1s     = state->q_n_1s().cviewer().name("q_n_1s"),
                    q_v_ns     = info.q_vs().viewer().name("q_v_ns"),
                    q_v_n_1s   = state->q_v_n_1s().cviewer().name("q_v_n_1s"),
                    q_tildes   = info.q_tildes().viewer().name("q_tilde"),
                    gravity    = info.gravities().cviewer().name("affine_gravity"),
                    dt         = info.dt()] __device__(int i) mutable
                   {
                       // q_n is tracked in q_prevs for current step.
                       q_ns(i) = qs(i);
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
                   });
    }

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace BDF2 = sym::abd_bdf2;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [qs       = info.qs().cviewer().name("qs"),
                    q_ns     = info.q_prevs().cviewer().name("q_n"),
                    q_n_1s   = state->q_n_1s().viewer().name("q_n_1s"),
                    q_v_ns   = info.q_vs().viewer().name("q_v_ns"),
                    q_v_n_1s = state->q_v_n_1s().viewer().name("q_v_n_1s"),
                    dt       = info.dt()] __device__(int i) mutable
                   {
                       Vector12 qv;
                       const auto& q     = qs(i);
                       const auto& q_n   = q_ns(i);
                       auto&    q_n_1 = q_n_1s(i);
                       BDF2::compute_qv(qv, q, q_n, q_n_1, dt);

                       // Update velocity history.
                       q_v_n_1s(i) = q_v_ns(i);
                       q_v_ns(i)   = qv;

                       // Update position history.
                       q_n_1s(i) = q_n;
                   });
    }
};

REGISTER_SIM_SYSTEM(ABDBDF2Integrator);
}  // namespace uipc::backend::cuda
