#include <finite_element/finite_element_kinetic.h>
#include <time_integrator/bdf2_flag.h>

namespace uipc::backend::cuda
{
class FiniteElementBDF2Kinetic final : public FiniteElementKinetic
{
  public:
    static constexpr Float beta     = 4.0 / 9.0;  // BDF2 beta coefficient
    static constexpr Float inv_beta = Float{1} / beta;

    using FiniteElementKinetic::FiniteElementKinetic;

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
            .apply(info.xs().size(),
                   [is_fixed = info.is_fixed().cviewer(),
                    xs       = info.xs().cviewer(),
                    x_tildes = info.x_tildes().cviewer(),
                    masses   = info.masses().cviewer(),
                    Ks       = info.energies().viewer(),
                    inv_beta = inv_beta] __device__(int i) mutable
                   {
                       auto& K = Ks(i);

                       if(is_fixed(i))
                       {
                           K = 0.0;
                       }
                       else
                       {
                           const auto& x       = xs(i);
                           const auto& x_tilde = x_tildes(i);
                           const auto& M       = masses(i);
                           const auto  dx      = x - x_tilde;

                           K = inv_beta / 2 * dx.dot(M * dx);
                       }
                   });
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.xs().size(),
                   [is_fixed      = info.is_fixed().cviewer(),
                    xs            = info.xs().cviewer(),
                    x_tildes      = info.x_tildes().viewer(),
                    masses        = info.masses().cviewer(),
                    G3s           = info.gradients().viewer(),
                    H3x3s         = info.hessians().viewer(),
                    gradient_only = info.gradient_only(),
                    inv_beta      = inv_beta] __device__(int i) mutable
                   {
                       auto& m       = masses(i);
                       auto& x       = xs(i);
                       auto& x_tilde = x_tildes(i);

                       Vector3 G;

                       if(is_fixed(i))
                       {
                           G = Vector3::Zero();
                       }
                       else
                       {
                           G = inv_beta * m * (x - x_tilde);
                       }

                       G3s(i).write(i, G);

                       if(gradient_only)
                           return;

                       const Matrix3x3 H = (inv_beta * masses(i)) * Matrix3x3::Identity();
                       H3x3s(i).write(i, i, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(FiniteElementBDF2Kinetic);
}  // namespace uipc::backend::cuda
