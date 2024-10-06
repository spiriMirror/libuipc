#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <numbers>

namespace uipc::backend::cuda
{
class Empty2D final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 0ull;

    using Codim2DConstitution::Codim2DConstitution;

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override
    {
        // do nothing
    }

    virtual void do_retrieve(FiniteElementMethod::Codim2DFilteredInfo& info) override
    {
        // do nothing
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        info.element_energies().fill(0);
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        info.gradient().fill(Vector9::Zero());
        info.hessian().fill(Matrix9x9::Zero());
    }
};

REGISTER_SIM_SYSTEM(Empty2D);
}  // namespace uipc::backend::cuda
