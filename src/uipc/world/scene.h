#pragma once
#include <uipc/world/contact_tabular.h>

namespace uipc::world
{
class Scene
{
  public:
    Scene() = default;

    ContactTabular&       contact_tabular();
    const ContactTabular& contact_tabular() const;

  private:
    ContactTabular m_contact_tabular;
};
}  // namespace uipc::world
