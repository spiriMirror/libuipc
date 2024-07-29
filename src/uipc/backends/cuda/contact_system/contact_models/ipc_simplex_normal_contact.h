#pragma once
#include <contact_system/simplex_normal_contact.h>

namespace uipc::backend::cuda
{
class IPCSimplexNormalContact final : public SimplexNormalContact
{
  public:
    using SimplexNormalContact::SimplexNormalContact;

    virtual void do_compute_energy(EnergyInfo& info) override;
    virtual void do_build(BuildInfo& info) override;
    virtual void do_assemble(ContactInfo& info) override;
};
}  // namespace uipc::backend::cuda