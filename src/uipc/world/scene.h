#pragma once
#include <uipc/world/contact_tabular.h>
#include <uipc/world/constitution_tabular.h>

namespace uipc::world
{
class Scene
{
  public:
    Scene() = default;

    ContactTabular&       contact_tabular() noexcept;
    const ContactTabular& contact_tabular() const noexcept;

    ConstitutionTabular&       constitution_tabular() noexcept;
    const ConstitutionTabular& constitution_tabular() const noexcept;

  private:
    ContactTabular      m_contact_tabular;
    ConstitutionTabular m_constitution_tabular;
};
}  // namespace uipc::world
