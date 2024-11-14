#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
class Particle0D final : public Codim0DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 13;

    using Codim0DConstitution::Codim0DConstitution;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override
    {
        // Do nothing
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        // Do nothing
    }
};

REGISTER_SIM_SYSTEM(Particle0D);
}  // namespace uipc::backend::cuda
