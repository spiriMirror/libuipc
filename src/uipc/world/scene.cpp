#include <uipc/world/scene.h>

namespace uipc::world
{
ContactTabular& Scene::contact_tabular()
{
    return m_contact_tabular;
}

const ContactTabular& Scene::contact_tabular() const
{
    return m_contact_tabular;
}
}  // namespace uipc::world
