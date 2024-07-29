#pragma once
#include <contact_system/simplex_frictional_contact.h>

namespace uipc::backend::cuda
{
class IPCSimplexFrictionalContact final : public SimplexFrictionalContact
{
  public:
    using SimplexFrictionalContact::SimplexFrictionalContact;

    virtual void do_compute_energy(EnergyInfo& info) override;
    virtual void do_build(BuildInfo& info) override;
    virtual void do_assemble(ContactInfo& info) override;
};
}  // namespace uipc::backend::cuda