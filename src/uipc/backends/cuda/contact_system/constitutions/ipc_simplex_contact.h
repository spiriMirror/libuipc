#pragma once
#include <contact_system/simplex_contact_constitution.h>

namespace uipc::backend::cuda
{
class IPCSimplexContact : public SimplexContactConstitution
{
  public:
    using SimplexContactConstitution::SimplexContactConstitution;

    class Impl;

    class Impl
    {
    };

  private:
    Impl m_impl;

    // Inherited via SimplexContactConstitution
    void do_compute_energy(GlobalContactManager::EnergyInfo& info) override;
    void do_build(BuildInfo& info) override;
    void do_report_count(PrimitiveCountInfo& info) override;
    void do_assemble(ContactInfo& info) override;
};
}  // namespace uipc::backend::cuda