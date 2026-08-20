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
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [is_fixed   = info.is_fixed().cviewer(),
                    is_dynamic = info.is_dynamic().cviewer(),
                    x_prevs    = info.x_prevs().viewer(),
                    xs         = info.xs().cviewer(),
                    vs         = info.vs().cviewer(),
                    x_tildes   = info.x_tildes().viewer(),
                    gravities  = info.gravities().cviewer(),
                    external_force_accs = info.external_force_accs().cviewer(),
                    dt         = info.dt()] __device__(int i) mutable
                   {
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
                   });
    }

    virtual void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [xs      = info.xs().cviewer(),
                    vs      = info.vs().viewer(),
                    x_prevs = info.x_prevs().cviewer(),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector3&       v      = vs(i);
                       const Vector3& x_prev = x_prevs(i);
                       const Vector3& x      = xs(i);

                       v = (x - x_prev) * (1.0 / dt);
                   });

    }
};

REGISTER_SIM_SYSTEM(FEMBDF1Integrator);
}  // namespace uipc::backend::cuda
