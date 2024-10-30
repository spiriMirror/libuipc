#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <numbers>

namespace uipc::backend::cuda
{
class Empty3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 0ull;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float>             h_kappas;
    muda::DeviceBuffer<Float> kappas;
    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_report_extent(ReportExtentInfo& info)
    {
        info.energy_count(0);
        info.stencil_dim(4);
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

REGISTER_SIM_SYSTEM(Empty3D);
}  // namespace uipc::backend::cuda
