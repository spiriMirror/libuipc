#include <uipc/world/scene.h>

namespace uipc::world
{
// ----------------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------------
Json Scene::default_config() noexcept
{
    Json j;
    j["dt"]                           = 0.01;
    j["gravity"]                      = Vector3{0.0, -9.8, 0.0};
    j["newton"]["tolerance"]          = 1e-2;
    j["newton"]["max_iter"]           = 1000;
    j["linear_system"]["solver"]      = "linear_pcg";
    j["line_search"]["report_energy"] = false;
    j["contact"]["enable"]            = true;
    j["contact"]["contitution"]       = "ipc";
    j["contact"]["d_hat"]             = 0.01;
    // something that is unofficial
    j["extras"] = Json::object();
    return j;
}

Scene::Scene(const Json& config)
{
    m_info = config;
}

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

auto Scene::objects() noexcept -> Objects
{
    return Objects{*this};
}

auto Scene::objects() const noexcept -> CObjects
{
    return CObjects{*this};
}

auto Scene::geometries() noexcept -> Geometries
{
    return Geometries{*this};
}

auto Scene::geometries() const noexcept -> CGeometries
{
    return CGeometries{*this};
}

const Json& Scene::info() const noexcept
{
    return m_info;
}

void Scene::solve_pending() noexcept
{
    m_geometries.solve_pending();
    m_rest_geometries.solve_pending();
}

// ----------------------------------------------------------------------------
// Objects
// ----------------------------------------------------------------------------
P<Object> Scene::Objects::create(std::string_view name) &&
{
    auto id = m_scene.m_objects.m_next_id;
    return m_scene.m_objects.emplace(Object{m_scene, id, name});
}

P<Object> Scene::Objects::find(IndexT id) && noexcept
{
    return m_scene.m_objects.find(id);
}

void Scene::Objects::destroy(IndexT id) &&
{
    auto obj = m_scene.m_objects.find(id);
    if(!obj)
    {
        UIPC_WARN_WITH_LOCATION("Trying to destroy non-existing object ({}), ignored.", id);
        return;
    }

    auto geo_ids = obj->geometries().ids();

    for(auto geo_id : geo_ids)
    {
        if(!m_scene.m_started)
        {
            m_scene.m_geometries.destroy(geo_id);
            m_scene.m_rest_geometries.destroy(geo_id);
        }
        else
        {
            m_scene.m_geometries.pending_destroy(geo_id);
            m_scene.m_rest_geometries.pending_destroy(geo_id);
        }
    }
    m_scene.m_objects.destroy(id);
}

SizeT Scene::Objects::size() const noexcept
{
    return m_scene.m_objects.size();
}

P<const Object> Scene::CObjects::find(IndexT id) && noexcept
{
    return m_scene.m_objects.find(id);
}

SizeT Scene::CObjects::size() const noexcept
{
    return m_scene.m_objects.size();
}

Scene::Objects::Objects(Scene& scene) noexcept
    : m_scene{scene}
{
}

Scene::CObjects::CObjects(const Scene& scene) noexcept
    : m_scene{scene}
{
}

// ----------------------------------------------------------------------------
// Geometries
// ----------------------------------------------------------------------------
Scene::Geometries::Geometries(Scene& scene) noexcept
    : m_scene{scene}
{
}

Scene::CGeometries::CGeometries(const Scene& scene) noexcept
    : m_scene{scene}
{
}
}  // namespace uipc::world
