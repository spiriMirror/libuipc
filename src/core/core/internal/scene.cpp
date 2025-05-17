#include <uipc/core/internal/scene.h>

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

Scene::~Scene() = default;
}  // namespace uipc::core::internal