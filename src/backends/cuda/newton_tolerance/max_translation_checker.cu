#include <newton_tolerance/newton_tolerance_checker.h>
#include <global_geometry/global_vertex_manager.h>

namespace uipc::backend::cuda
{
class MaxTranslationChecker : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<GlobalVertexManager> vertex_manager;
    Float                              res0    = 0.0;
    Float                              res     = 0.0;
    Float                              rel_tol = 0.0;
    Float                              abs_tol = 0.0;
    SizeT                              frame   = 0;

    void do_build(BuildInfo& info) override
    {
        vertex_manager            = require<GlobalVertexManager>();
        auto& config              = world().scene().info();
        Float dt                  = config["dt"];
        Float newton_velocity_tol = config["newton"]["velocity_tol"];

        abs_tol = newton_velocity_tol * dt;
    }
    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override
    {
        res0  = 0.0;
        frame = info.frame();
    }

    void do_check(CheckResultInfo& info) override
    {
        res              = vertex_manager->compute_axis_max_displacement();
        auto newton_iter = info.newton_iter();
        if(newton_iter == 0)
            res0 = res;  // record the initial residual

        rel_tol = res == 0.0 ? 0.0 : res / res0;

        auto converged = res <= abs_tol || rel_tol <= 0.001;

        info.converged(converged);
    }

    std::string do_report() override
    {
        std::string report = fmt::format("Residual/AbsTol: {}/{}", res, abs_tol);
        return report;
    }
};

REGISTER_SIM_SYSTEM(MaxTranslationChecker);
}  // namespace uipc::backend::cuda