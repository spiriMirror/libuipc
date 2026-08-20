#include <newton_tolerance/newton_tolerance_checker.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_tolerance_checker_do_check_kernel(
        cuda_tool::CBufferView<Vector12> dqs,
        cuda_tool::VarView<IndexT>       success,
        Float                            abs_tol,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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
                cuda_tool::atomic_exch(success.data(), 0);
                break;  // no need to check further
            }
        }
    }
}  // namespace

class ABDToleranceChecker final : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
    S<const geometry::AttributeSlot<Float>> dt_attr;
    Float                                   transrate_tol = 0.0;
    Float                                   abs_tol       = 0.0;
    cuda_tool::DeviceVar<IndexT>                 success;
    IndexT h_success = 1;  // 1 means success, 0 means failure

    // Inherited via NewtonToleranceChecker
    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        auto& config         = world().scene().config();
        dt_attr              = config.find<Float>("dt");
        UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
        auto transrate_tol_attr = config.find<Float>("newton/transrate_tol");
        transrate_tol           = transrate_tol_attr->view()[0];
    }

    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override {}

    void do_check(CheckResultInfo& info) override
    {
        abs_tol  = transrate_tol * dt_attr->view()[0];
        auto dqs = affine_body_dynamics->dqs();
        using namespace cuda_tool;
        BufferLaunch().fill(success.view(), 1);  // reset success flag

        int n = (int)dqs.size();
        if(n > 0)
        {
            auto k = abd_tolerance_checker_do_check_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                dqs, success.view(), abs_tol, n);
        }

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