#pragma once
#include <contact_system/simplex_contact_constitution.h>

namespace uipc::backend::cuda
{
class IPCSimplexContact final : public SimplexContactConstitution
{
  public:
    using SimplexContactConstitution::SimplexContactConstitution;

    virtual void do_compute_energy(EnergyInfo& info) override;
    virtual void do_build(BuildInfo& info) override;
    virtual void do_assemble(ContactInfo& info) override;
};
}  // namespace uipc::backend::cuda