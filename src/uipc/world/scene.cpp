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

auto Scene::objects() -> Objects
{
    return Objects{*this};
}

const set<IndexT>& Scene::pending_destroy() noexcept
{
    return m_objects.m_pending_destroy;
}

const unordered_map<IndexT, S<ObjectSlot>>& Scene::pending_create() noexcept
{
    return m_objects.m_pending_create;
}

void Scene::solve_pending() noexcept
{
    m_objects.solve_pending();
}

P<ObjectSlot> Scene::Objects::create(std::string_view name) &&
{
    return std::move(*this).create(Object{name});
}

P<ObjectSlot> Scene::Objects::create(Object&& object) &&
{
    return m_scene.m_is_running ? m_scene.m_objects.pending_emplace(std::move(object)) :
                                  m_scene.m_objects.emplace(std::move(object));
}

P<ObjectSlot> Scene::Objects::find(IndexT id) && noexcept
{
    return m_scene.m_objects.find(id);
}

void Scene::Objects::destroy(IndexT id) &&
{
    return m_scene.m_is_running ? m_scene.m_objects.pending_destroy(id) :
                                  m_scene.m_objects.destroy(id);
}

P<const ObjectSlot> Scene::CObjects::find(IndexT id) && noexcept
{
    return m_scene.m_objects.find(id);
}

Scene::Objects::Objects(Scene& scene) noexcept
    : m_scene{scene}
{
}

Scene::CObjects::CObjects(const Scene& scene) noexcept
    : m_scene{scene}
{
}


}  // namespace uipc::world
