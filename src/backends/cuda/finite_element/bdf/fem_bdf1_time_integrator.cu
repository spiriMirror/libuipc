#include <finite_element/fem_time_integrator.h>
#include <time_integrator/bdf1_flag.h>

namespace uipc::backend::cuda
{
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
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [is_fixed   = info.is_fixed().cviewer().name("fixed"),
                    is_dynamic = info.is_dynamic().cviewer().name("is_dynamic"),
                    x_prevs    = info.x_prevs().cviewer().name("x_prevs"),
                    vs         = info.vs().cviewer().name("vs"),
                    x_tildes   = info.x_tildes().viewer().name("x_tildes"),
                    gravities  = info.gravities().cviewer().name("gravities"),
                    dt         = info.dt()] __device__(int i) mutable
                   {
                       const Vector3& x_prev = x_prevs(i);
                       const Vector3& v      = vs(i);

                       // 0) fixed: x_tilde = x_prev
                       Vector3 x_tilde = x_prev;

                       if(!is_fixed(i))
                       {
                           const Vector3& g = gravities(i);

                           // 1) static problem: x_tilde = x_prev + g * dt * dt
                           x_tilde += g * dt * dt;

                           // 2) dynamic problem: x_tilde = x_prev + v * dt + g * dt * dt
                           if(is_dynamic(i))
                           {
                               x_tilde += v * dt;
                           }
                       }

                       x_tildes(i) = x_tilde;
                   });
    }

    virtual void do_update_state(UpdateStateInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [xs      = info.xs().cviewer().name("xs"),
                    vs      = info.vs().viewer().name("vs"),
                    x_prevs = info.x_prevs().viewer().name("x_prevs"),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector3&       v      = vs(i);
                       Vector3&       x_prev = x_prevs(i);
                       const Vector3& x      = xs(i);

                       v = (x - x_prev) * (1.0 / dt);

                       x_prev = x;
                   });
    }
};

REGISTER_SIM_SYSTEM(FEMBDF1Integrator);
}  // namespace uipc::backend::cuda
