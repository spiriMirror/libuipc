#include <finite_element/codim_1d_constitution.h>

namespace uipc::backend::cuda
{
class Empty1D final : public Codim1DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 0ull;

    using Codim1DConstitution::Codim1DConstitution;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_report_extent(ReportExtentInfo& info)
    {
        info.energy_count(0);
        info.stencil_dim(2);
    }

    virtual void do_build(BuildInfo& info) override
    {
        // do nothing
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        // do nothing
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        // do nothing
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        // do nothing
    }
};

REGISTER_SIM_SYSTEM(Empty1D);
}  // namespace uipc::backend::cuda
