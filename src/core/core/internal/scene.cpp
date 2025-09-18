#include <uipc/core/internal/scene.h>
#include <uipc/core/scene_snapshot.h>

namespace uipc::core::internal
{
Scene::Scene(const Json& config) noexcept
    : m_animator{*this}
    , m_sanity_checker(*this)
{
    build_config(config);
}

void Scene::init(internal::World& world) noexcept
{
    m_world = &world;

    m_constitution_tabular.init(*this);

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
void create_and_set(geometry::AttributeCollection& coll, std::string_view key, const T& value)
{

    auto attr = coll.find<T>(key);
    if(!attr)
        attr = coll.create<T>(key);
    view(*attr)[0] = value;
}

// For boolean values converted to IndexT
template <>
void create_and_set(geometry::AttributeCollection& coll, std::string_view key, const bool& value)
{
    auto attr = coll.find<IndexT>(key);
    if(!attr)
        attr = coll.create<IndexT>(key);
    view(*attr)[0] = value ? 1 : 0;
}

void Scene::build_config(const Json& config)
{
    using namespace uipc::geometry;

    auto b2i = [](bool b) { return b ? 1 : 0; };

    m_config.resize(1);

    // dt
    create_and_set(m_config, "dt", config["dt"].get<Float>());

    // gravity
    create_and_set(m_config, "gravity", config["gravity"].get<Vector3>());

    // cfl
    {
        // cfl/enable
        create_and_set(m_config, "cfl/enable", b2i(config["cfl"]["enable"].get<bool>()));
    }

    // integrator
    {
        // integrator/type
        create_and_set(m_config,
                       "integrator/type",
                       config["integrator"]["type"].get<std::string>());
    }

    // newton
    {
        auto& newton = config["newton"];
        // newton/max_iter
        create_and_set(m_config, "newton/max_iter", newton["max_iter"].get<IndexT>());

        // newton/use_adaptive_tol
        create_and_set(m_config,
                       "newton/use_adaptive_tol",
                       b2i(newton["use_adaptive_tol"].get<bool>()));

        // newton/velocity_tol
        create_and_set(m_config, "newton/velocity_tol", newton["velocity_tol"].get<Float>());

        // newton/ccd_tol
        create_and_set(m_config, "newton/ccd_tol", newton["ccd_tol"].get<Float>());

        // newton/transrate_tol
        create_and_set(m_config, "newton/transrate_tol", newton["transrate_tol"].get<Float>());
    }

    // linear_system
    {
        auto& linear_system = config["linear_system"];
        // linear_system/tol_rate
        create_and_set(m_config,
                       "linear_system/tol_rate",
                       linear_system["tol_rate"].get<Float>());

        // linear_system/solver
        create_and_set(m_config,
                       "linear_system/solver",
                       linear_system["solver"].get<std::string>());
    }

    // line_search
    {
        auto& line_search = config["line_search"];
        // line_search/max_iter
        create_and_set(m_config, "line_search/max_iter", line_search["max_iter"].get<IndexT>());

        // line_search/report_energy
        create_and_set(m_config,
                       "line_search/report_energy",
                       b2i(line_search["report_energy"].get<bool>()));
    }

    // contact
    {
        auto& contact = config["contact"];
        // contact/enable
        create_and_set(m_config, "contact/enable", b2i(contact["enable"].get<bool>()));

        // contact/friction/enable
        create_and_set(m_config,
                       "contact/friction/enable",
                       b2i(contact["friction"]["enable"].get<bool>()));

        // contact/constitution
        create_and_set(m_config,
                       "contact/constitution",
                       contact["constitution"].get<std::string>());

        // contact/d_hat
        create_and_set(m_config, "contact/d_hat", contact["d_hat"].get<Float>());

        // contact/eps_velocity
        create_and_set(m_config, "contact/eps_velocity", contact["eps_velocity"].get<Float>());
    }

    // collision_detection
    {
        auto& collision_detection = config["collision_detection"];
        // collision_detection/method
        create_and_set(m_config,
                       "collision_detection/method",
                       collision_detection["method"].get<std::string>());
    }

    // sanity_check
    {
        auto& sanity_check = config["sanity_check"];
        // sanity_check/enable
        create_and_set(m_config,
                       "sanity_check/enable",
                       b2i(sanity_check["enable"].get<bool>()));

        // sanity_check/mode
        create_and_set(m_config, "sanity_check/mode", sanity_check["mode"].get<std::string>());
    }

    // recovery
    {
        // now just empty
    }

    // diff_sim
    {
        auto& diff_sim = config["diff_sim"];
        // diff_sim/enable
        create_and_set(m_config, "diff_sim/enable", b2i(diff_sim["enable"].get<bool>()));
    }

    // extras
    {
        auto& extras = config["extras"];
        // extras/debug/dump_surface
        create_and_set(m_config,
                       "extras/debug/dump_surface",
                       b2i(extras["debug"]["dump_surface"].get<bool>()));

        // extras/strict_mode/enable
        create_and_set(m_config,
                       "extras/strict_mode/enable",
                       b2i(extras["strict_mode"]["enable"].get<bool>()));
    }
}

Scene::~Scene() {}
}  // namespace uipc::core::internal