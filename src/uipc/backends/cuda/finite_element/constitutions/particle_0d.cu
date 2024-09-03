#include <finite_element/codim_0d_constitution.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>
#include <utils/codim_thickness.h>
#include <numbers>

namespace uipc::backend::cuda
{
class Particle0D final : public Codim0DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 13;

    using Codim0DConstitution::Codim0DConstitution;

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override
    {
        // Do nothing
    }

    virtual void do_retrieve(FiniteElementMethod::Codim0DFilteredInfo& info) override
    {
        // Do nothing
    }
};

REGISTER_SIM_SYSTEM(Particle0D);
}  // namespace uipc::backend::cuda
