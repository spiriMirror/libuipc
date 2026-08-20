#include <affine_body/affine_body_kinetic.h>
#include <time_integrator/bdf1_flag.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
class AffineBodyBDF1Kinetic final : public AffineBodyKinetic
{
  public:
    using AffineBodyKinetic::AffineBodyKinetic;

    virtual void do_build(BuildInfo& info) override
    {
        // need BDF1 flag for BDF1 time integration
        require<BDF1Flag>();
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [is_fixed   = info.is_fixed().cviewer(),
                    is_dynamic = info.is_dynamic().cviewer(),
                    ext_kinetic = info.external_kinetic().cviewer(),
                    qs        = info.qs().cviewer(),
                    q_prevs   = info.q_prevs().cviewer(),
                    q_tildes  = info.q_tildes().cviewer(),
                    gravities = info.gravities().cviewer(),
                    masses    = info.masses().cviewer(),
                    Ks = info.energies().viewer()] __device__(int i) mutable
                   {
                       auto& K = Ks(i);
                       if(is_fixed(i) || ext_kinetic(i))
                       {
                           K = 0.0;
                       }
                       else
                       {
                           const auto& q       = qs(i);
                           const auto& q_tilde = q_tildes(i);
                           const auto& M       = masses(i);
                           Vector12    dq      = q - q_tilde;
                           K                   = 0.5 * dq.dot(M * dq);
                       }
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [is_fixed   = info.is_fixed().cviewer(),
                    is_dynamic = info.is_dynamic().cviewer(),
                    qs         = info.qs().cviewer(),
                    q_prevs    = info.q_prevs().cviewer(),
                    q_tildes   = info.q_tildes().cviewer(),
                    gravities  = info.gravities().cviewer(),
                    masses     = info.masses().cviewer(),
                    hessians   = info.hessians().viewer(),
                    gradients  = info.gradients().viewer(),
                    dt         = info.dt(),
                    gradient_only = info.gradient_only(),
                    cout = KernelCout::viewer()] __device__(int i) mutable
                   {
                       const auto& q       = qs(i);
                       const auto& q_prev  = q_prevs(i);
                       const auto& q_tilde = q_tildes(i);
                       auto&       G       = gradients(i);
                       const auto& M       = masses(i);

                       G = M * (q - q_tilde);


                       if(is_fixed(i))
                       {
                           G = Vector12::Zero();
                       }

                       // cout << "KG(" << i << "): " << G.transpose().eval() << "\n";

                       if(gradient_only)
                           return;

                       hessians(i) = M.to_mat();
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyBDF1Kinetic);
}  // namespace uipc::backend::cuda
