#include <newton_tolerance/newton_tolerance_checker.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class ABDToleranceChecker final : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
    Float                             abs_tol = 0.0;
    muda::DeviceVar<IndexT>           success;
    IndexT h_success = 1;  // 1 means success, 0 means failure
    
    // Inherited via NewtonToleranceChecker
    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        auto& config         = world().scene().info();
        Float dt             = config["dt"].get<Float>();
        Float transrate_tol  = config["newton"]["transrate_tol"].get<Float>();
        abs_tol              = transrate_tol * dt;
    }

    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override {}

    void do_check(CheckResultInfo& info) override
    {
        auto dqs = affine_body_dynamics->dqs();
        using namespace muda;
        BufferLaunch().fill(success.view(), 1);  // reset success flag

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(dqs.size(),
                   [dqs     = dqs.viewer().name("dqs"),
                    success = success.viewer().name("success"),
                    abs_tol = abs_tol] __device__(int I)
                   {
                       const Vector12& dq            = dqs(I);
                       IndexT          success_value = *success;

                       // if success is already marked as failed, skip
                       if(success_value == 0)
                           return;

                       // the first 3 components are translation, ignore
                       // the rest 9 components are rotation/scaling/shear, take
                       for(IndexT i = 3; i < 12; ++i)
                       {
                           if(abs(dq[i]) > abs_tol)
                           {
                               muda::atomic_exch(success.data(), 0);
                               break;  // no need to check further
                           }
                       }
                   });

        // copy from device to host
        bool h_success = success;
        info.converged(h_success);
    }

    std::string do_report() override
    {
        return fmt::format("Tol: {}{}", (h_success ? "< " : "> "), abs_tol);
    }
};

REGISTER_SIM_SYSTEM(ABDToleranceChecker);
}  // namespace uipc::backend::cuda