#include <newton_tolerance/newton_tolerance_checker.h>
#include <global_geometry/global_vertex_manager.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::backend::cuda
{
class MaxTranslationChecker : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<GlobalVertexManager>      vertex_manager;
    S<const geometry::AttributeSlot<Float>> dt_attr;
    Float                                   newton_velocity_tol = 0.0;
    Float                                   res0                = 0.0;
    Float                                   res                 = 0.0;
    Float                                   rel_tol             = 0.0;
    Float                                   abs_tol             = 0.0;
    SizeT                                   frame               = 0;

    void do_build(BuildInfo& info) override
    {
        vertex_manager = require<GlobalVertexManager>();
        auto& config   = world().scene().config();
        dt_attr        = config.find<Float>("dt");
        UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
        auto newton_velocity_tol_attr = config.find<Float>("newton/velocity_tol");
        newton_velocity_tol = newton_velocity_tol_attr->view()[0];
    }
    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override
    {
        res0  = 0.0;
        frame = info.frame();
    }

    void do_check(CheckResultInfo& info) override
    {
        abs_tol          = newton_velocity_tol * dt_attr->view()[0];
        res              = vertex_manager->compute_axis_max_displacement();
        auto newton_iter = info.newton_iter();
        if(newton_iter == 0)
            res0 = res;  // record the initial residual

        auto converged = res <= abs_tol;

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