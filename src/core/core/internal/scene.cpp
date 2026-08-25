#include <uipc/core/internal/scene.h>
#include <uipc/core/scene_snapshot.h>

namespace uipc::core::internal
{
Scene::Scene(const geometry::AttributeCollection& config) noexcept
    : m_animator{*this}
    , m_config{config}
{
}

void Scene::init(internal::World& world) noexcept
{
    m_world = &world;

    m_constitution_tabular.init(*this);

    // If differentiable simulation is enabled initialize it
    auto diff_sim_enable = m_config.find<IndexT>("diff_sim/enable");
    if(diff_sim_enable->view()[0])
    {
        m_diff_sim.init(*this);
    }

    m_started = true;
}

void Scene::begin_pending() noexcept
{
    m_pending = true;
}

void Scene::solve_pending()
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

    m_config.update_from(*commit.m_config);

    m_objects.update_from(*this, commit.m_object_collection);
    m_contact_tabular.update_from(*commit.m_contact_models,
                                  commit.m_contact_elements,
                                  commit.m_contact_default_model_user_set);
    m_subscene_tabular.update_from(*commit.m_subscene_models, commit.m_subscene_elements);

    m_geometries.update_from(commit.m_geometries,
                             commit.m_removed_geometry_ids,
                             commit.m_geometry_next_id);
    m_rest_geometries.update_from(commit.m_rest_geometries,
                                  commit.m_removed_rest_geometry_ids,
                                  commit.m_rest_geometry_next_id);
}

Float Scene::dt() const
{
    auto dt_attr = m_config.find<Float>("dt");
    UIPC_ASSERT_THROW(dt_attr, "Scene config must have a 'dt' attribute.");
    return dt_attr->view()[0];
}

Scene::~Scene() = default;
}  // namespace uipc::core::internal
