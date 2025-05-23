#include <uipc/core/internal/scene.h>
#include <uipc/core/scene_snapshot.h>

namespace uipc::core::internal
{
Scene::Scene(const Json& config) noexcept
    : m_animator{*this}
    , m_sanity_checker(*this)
{
    m_config = config;
}

void Scene::init(internal::World& world) noexcept
{
    m_world = &world;

    m_dt = m_config["dt"].get<Float>();

    m_constitution_tabular.init(*this);

    if(m_config["diff_sim"]["enable"].get<bool>())
    {
        m_diff_sim.init(*this);
    }

    m_started = true;
}

void Scene::begin_pending() noexcept
{
    m_pending = true;
}

void Scene::solve_pending() noexcept
{
    m_geometries.solve_pending();
    m_rest_geometries.solve_pending();
    m_pending = false;
}

void Scene::update_from(const SceneSnapshotCommit& commit)
{
    if(!commit.is_valid())
    {
        UIPC_WARN_WITH_LOCATION("Invalid scene commit, scene will not be updated.");
        return;
    }

    m_config = commit.m_config;
    m_objects.update_from(*this, commit.m_object_collection);
    m_contact_tabular.update_from(*commit.m_contact_models, commit.m_contact_elements);

    m_geometries.update_from(commit.m_geometries);
    m_rest_geometries.update_from(commit.m_rest_geometries);
}

Scene::~Scene() = default;
}  // namespace uipc::core::internal