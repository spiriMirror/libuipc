#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
class Empty0D final : public Codim0DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 0;

    using Codim0DConstitution::Codim0DConstitution;

    virtual U64 get_uid() const override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override
    {
        // Do nothing
    }

    virtual void do_init(FiniteElementMethod::Codim0DFilteredInfo& info) override
    {
        // Do nothing
    }
};

REGISTER_SIM_SYSTEM(Empty0D);
}  // namespace uipc::backend::cuda
