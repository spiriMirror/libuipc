#include <uipc/world/scene.h>

namespace uipc::world
{
ContactTabular& Scene::contact_tabular() noexcept
{
    return m_contact_tabular;
}

const ContactTabular& Scene::contact_tabular() const noexcept
{
    return m_contact_tabular;
}
ConstitutionTabular& Scene::constitution_tabular() noexcept
{
    return m_constitution_tabular;
}
const ConstitutionTabular& Scene::constitution_tabular() const noexcept
{
    return m_constitution_tabular;
}
}  // namespace uipc::world
