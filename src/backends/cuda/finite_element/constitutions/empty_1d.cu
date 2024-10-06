#include <finite_element/codim_1d_constitution.h>

namespace uipc::backend::cuda
{
class Empty1D final : public Codim1DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 0ull;

    using Codim1DConstitution::Codim1DConstitution;

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override
    {
        // do nothing
    }

    virtual void do_retrieve(FiniteElementMethod::Codim1DFilteredInfo& info) override
    {
        // do nothing
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        info.element_energies().fill(0);
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        info.gradient().fill(Vector6::Zero());
        info.hessian().fill(Matrix6x6::Zero());
    }
};

REGISTER_SIM_SYSTEM(Empty1D);
}  // namespace uipc::backend::cuda
