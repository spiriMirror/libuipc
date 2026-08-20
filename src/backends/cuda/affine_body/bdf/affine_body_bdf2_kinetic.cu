#include <affine_body/affine_body_kinetic.h>
#include <time_integrator/bdf2_flag.h>

namespace uipc::backend::cuda
{
class AffineBodyBDF2Kinetic final : public AffineBodyKinetic
{
  public:
    static constexpr Float beta     = 4.0 / 9.0;  // BDF2 beta coefficient
    static constexpr Float inv_beta = Float{1} / beta;

    using AffineBodyKinetic::AffineBodyKinetic;

    void do_build(BuildInfo& info) override
    {
        // need BDF2 flag for BDF2 time integration
        require<BDF2Flag>();
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [is_fixed    = info.is_fixed().cviewer(),
                    ext_kinetic = info.external_kinetic().cviewer(),
                    qs          = info.qs().cviewer(),
                    q_tildes    = info.q_tildes().cviewer(),
                    masses      = info.masses().cviewer(),
                    Ks          = info.energies().viewer(),
                    inv_beta    = inv_beta] __device__(int i) mutable
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
                           const auto  dq      = q - q_tilde;
                           K = (0.5 * inv_beta) * dq.dot(M * dq);
                       }
                   });
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.qs().size(),
                   [is_fixed      = info.is_fixed().cviewer(),
                    qs            = info.qs().cviewer(),
                    q_tildes      = info.q_tildes().cviewer(),
                    masses        = info.masses().cviewer(),
                    hessians      = info.hessians().viewer(),
                    gradients     = info.gradients().viewer(),
                    gradient_only = info.gradient_only(),
                    inv_beta      = inv_beta] __device__(int i) mutable
                   {
                       const auto& M       = masses(i);
                       const auto& q       = qs(i);
                       const auto& q_tilde = q_tildes(i);

                       auto& G = gradients(i);
                       G       = inv_beta * (M * (q - q_tilde));

                       if(is_fixed(i))
                       {
                           G = Vector12::Zero();
                       }

                       if(gradient_only)
                           return;

                       hessians(i) = inv_beta * M.to_mat();
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyBDF2Kinetic);
}  // namespace uipc::backend::cuda
