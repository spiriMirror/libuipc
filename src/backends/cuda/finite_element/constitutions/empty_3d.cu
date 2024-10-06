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

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override
    {
        // do nothing
    }

    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) override
    {
        // do nothing
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        info.element_energies().fill(0);
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        info.gradient().fill(Vector12::Zero());
        info.hessian().fill(Matrix12x12::Zero());
    }
};

REGISTER_SIM_SYSTEM(Empty3D);
}  // namespace uipc::backend::cuda
