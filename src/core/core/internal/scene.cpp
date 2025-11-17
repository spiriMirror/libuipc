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

    m_config.update_from(*commit.m_config);

    m_objects.update_from(*this, commit.m_object_collection);
    m_contact_tabular.update_from(*commit.m_contact_models, commit.m_contact_elements);

    m_geometries.update_from(commit.m_geometries);
    m_rest_geometries.update_from(commit.m_rest_geometries);
}

Float Scene::dt() const noexcept
{
    auto dt_attr = m_config.find<Float>("dt");
    UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
    return dt_attr->view()[0];
}

template <typename T>
void create_and_set(geometry::AttributeCollection& coll,
                    std::string_view               key,
                    const T&                       value,
                    const T&                       default_value = T{0})
{
    auto attr = coll.find<T>(key);
    if(!attr)
        attr = coll.create<T>(key, default_value);
    view(*attr)[0] = value;
}

template <typename T, int M, int N>
void create_and_set(geometry::AttributeCollection& coll,
                    std::string_view               key,
                    const Matrix<T, M, N>&         value,
                    const Matrix<T, M, N>& default_value = Matrix<T, M, N>::Zero())
{
    auto attr = coll.find<Matrix<T, M, N>>(key);
    if(!attr)
        attr = coll.create<Matrix<T, M, N>>(key, default_value);
    view(*attr)[0] = value;
}

Scene::~Scene() = default;
}  // namespace uipc::core::internal