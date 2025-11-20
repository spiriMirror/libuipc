#include <uipc/core/scene.h>
#include <uipc/common/unit.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/internal/scene.h>
#include <uipc/core/scene_snapshot.h>

// local include
// for default_scene_config, to_config_json, from_config_json
#include "scene_default_config.h"

namespace uipc::core
{
// ----------------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------------
Json Scene::default_config() noexcept
{
    return to_config_json(default_scene_config());
}

Scene::Scene(const Json& j)
{
    geometry::AttributeCollection config = default_scene_config();
    from_config_json(config, j);
    m_internal = uipc::make_shared<internal::Scene>(config);
}

Scene::Scene(S<internal::Scene> scene) noexcept
    : m_internal(std::move(scene))
{
}

Scene::CConfigAttributes Scene::config() const noexcept
{
    return CConfigAttributes{m_internal->config()};
}

Scene::ConfigAttributes Scene::config() noexcept
{
    return ConfigAttributes{m_internal->config()};
}

Scene::~Scene() = default;

ContactTabular& Scene::contact_tabular() noexcept
{
    return m_internal->contact_tabular();
}

const ContactTabular& Scene::contact_tabular() const noexcept
{
    return m_internal->contact_tabular();
}

SubsceneTabular& Scene::subscene_tabular() noexcept
{
    return m_internal->subscene_tabular();
}

const SubsceneTabular& Scene::subscene_tabular() const noexcept
{
    return m_internal->subscene_tabular();
}

ConstitutionTabular& Scene::constitution_tabular() noexcept
{
    return m_internal->constitution_tabular();
}
const ConstitutionTabular& Scene::constitution_tabular() const noexcept
{
    return m_internal->constitution_tabular();
}

auto Scene::objects() noexcept -> Objects
{
    return Objects{*m_internal};
}

auto Scene::objects() const noexcept -> CObjects
{
    return CObjects{*m_internal};
}

auto Scene::geometries() noexcept -> Geometries
{
    return Geometries{*m_internal};
}

auto Scene::geometries() const noexcept -> CGeometries
{
    return CGeometries{*m_internal};
}

Animator& Scene::animator()
{
    return m_internal->animator();
}

const Animator& Scene::animator() const
{
    return m_internal->animator();
}

DiffSim& Scene::diff_sim()
{
    // automatically enable diff_sim
    auto diff_sim_enable = m_internal->config().find<IndexT>("diff_sim/enable");
    UIPC_ASSERT(diff_sim_enable, "Scene config must have a 'diff_sim/enable' attribute.");
    geometry::view(*diff_sim_enable)[0] = 1;
    return m_internal->diff_sim();
}

const DiffSim& Scene::diff_sim() const
{
    return m_internal->diff_sim();
}

void Scene::update_from(const SceneSnapshotCommit& snapshot)
{
    m_internal->update_from(snapshot);
}

// ----------------------------------------------------------------------------
// Objects
// ----------------------------------------------------------------------------
S<Object> Scene::Objects::create(std::string_view name) &&
{
    auto id = m_scene.objects().m_next_id;
    return m_scene.objects().emplace(Object{m_scene, id, name});
}

S<Object> Scene::Objects::find(IndexT id) && noexcept
{
    return m_scene.objects().find(id);
}

vector<S<Object>> Scene::Objects::find(std::string_view name) && noexcept
{
    return m_scene.objects().find(name);
}

void Scene::Objects::destroy(IndexT id) &&
{
    auto obj = m_scene.objects().find(id);
    if(!obj)
    {
        UIPC_WARN_WITH_LOCATION("Trying to destroy non-existing object ({}), ignored.", id);
        return;
    }

    auto geo_ids = obj->geometries().ids();

    for(auto geo_id : geo_ids)
    {
        if(m_scene.is_started() || m_scene.is_pending())
        {
            m_scene.geometries().pending_destroy(geo_id);
            m_scene.rest_geometries().pending_destroy(geo_id);
        }
        else  // before `world.init(scene)` is called
        {
            m_scene.geometries().destroy(geo_id);
            m_scene.rest_geometries().destroy(geo_id);
        }
    }
    m_scene.objects().destroy(id);
}

SizeT Scene::Objects::size() const noexcept
{
    return m_scene.objects().size();
}

SizeT Scene::Objects::created_count() const noexcept
{
    return m_scene.objects().next_id();
}

S<const Object> Scene::CObjects::find(IndexT id) && noexcept
{
    return m_scene.objects().find(id);
}

vector<S<const Object>> Scene::CObjects::find(std::string_view name) && noexcept
{
    return std::as_const(m_scene.objects()).find(name);
}

SizeT Scene::CObjects::size() const noexcept
{
    return m_scene.objects().size();
}

SizeT Scene::CObjects::created_count() const noexcept
{
    return m_scene.objects().next_id();
}

Scene::Objects::Objects(internal::Scene& scene) noexcept
    : m_scene{scene}
{
}

Scene::CObjects::CObjects(const internal::Scene& scene) noexcept
    : m_scene{scene}
{
}

// ----------------------------------------------------------------------------
// Geometries
// ----------------------------------------------------------------------------
Scene::Geometries::Geometries(internal::Scene& scene) noexcept
    : m_scene{scene}
{
}

Scene::CGeometries::CGeometries(const internal::Scene& scene) noexcept
    : m_scene{scene}
{
}

ObjectGeometrySlots<geometry::Geometry> core::Scene::Geometries::find(IndexT id) && noexcept
{
    return {m_scene.geometries().find(id), m_scene.rest_geometries().find(id)};
}

ObjectGeometrySlots<const geometry::Geometry> Scene::CGeometries::find(IndexT id) && noexcept
{
    return {m_scene.geometries().find(id), m_scene.rest_geometries().find(id)};
}
}  // namespace uipc::core


namespace fmt
{
appender fmt::formatter<uipc::core::Scene>::format(const uipc::core::Scene& c,
                                                   format_context& ctx) const
{
    fmt::format_to(ctx.out(), "config:\n{}", c.m_internal->config());
    fmt::format_to(ctx.out(), "\n{}", c.m_internal->objects());
    fmt::format_to(ctx.out(), "\n{}", c.m_internal->animator());
    return ctx.out();
}
}  // namespace fmt