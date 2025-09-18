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

void Scene::build_config(const Json& config)
{
    using namespace uipc::geometry;

    auto b2i = [](bool b) { return b ? 1 : 0; };

    m_config.resize(1);

    // dt
    auto dt = m_config.create<Float>("dt");
    fmt::println("{}", config.dump(4));
    view(*dt)[0] = config["dt"].get<Float>();

    // gravity
    auto gravity      = m_config.create<Vector3>("gravity");
    view(*gravity)[0] = config["gravity"].get<Vector3>();

    // cfl
    {
        // cfl/enable
        auto cfl_enable      = m_config.create<IndexT>("cfl/enable");
        view(*cfl_enable)[0] = b2i(config["cfl"]["enable"].get<bool>());
    }

    // integrator
    {
        // integrator/type
        auto integrator_type = m_config.create<std::string>("integrator/type");
        view(*integrator_type)[0] = config["integrator"]["type"].get<std::string>();
    }


    // newton
    {
        auto& newton = config["newton"];
        // newton/max_iter
        auto newton_max_iter      = m_config.create<IndexT>("newton/max_iter");
        view(*newton_max_iter)[0] = newton["max_iter"].get<IndexT>();

        // newton/use_adaptive_tol
        auto newton_use_adaptive_tol = m_config.create<IndexT>("newton/use_adaptive_tol");
        view(*newton_use_adaptive_tol)[0] = b2i(newton["use_adaptive_tol"].get<bool>());

        // newton/velocity_tol
        auto newton_velocity_tol = m_config.create<Float>("newton/velocity_tol");
        view(*newton_velocity_tol)[0] = newton["velocity_tol"].get<Float>();

        // newton/ccd_tol
        auto newton_ccd_tol      = m_config.create<Float>("newton/ccd_tol");
        view(*newton_ccd_tol)[0] = newton["ccd_tol"].get<Float>();

        // newton/transrate_tol
        auto newton_transrate_tol = m_config.create<Float>("newton/transrate_tol");
        view(*newton_transrate_tol)[0] = newton["transrate_tol"].get<Float>();
    }

    // linear_system
    {
        auto& linear_system = config["linear_system"];
        // linear_system/tol_rate
        auto linear_system_tol_rate = m_config.create<Float>("linear_system/tol_rate");
        view(*linear_system_tol_rate)[0] = linear_system["tol_rate"].get<Float>();

        // linear_system/solver
        auto linear_system_solver = m_config.create<std::string>("linear_system/solver");
        view(*linear_system_solver)[0] = linear_system["solver"].get<std::string>();
    }

    // line_search
    {
        auto& line_search = config["line_search"];
        // line_search/max_iter
        auto line_search_max_iter = m_config.create<IndexT>("line_search/max_iter");
        view(*line_search_max_iter)[0] = line_search["max_iter"].get<IndexT>();

        // line_search/report_energy
        auto line_search_report_energy =
            m_config.create<IndexT>("line_search/report_energy");
        view(*line_search_report_energy)[0] =
            b2i(line_search["report_energy"].get<bool>());
    }

    // contact
    {
        auto& contact = config["contact"];
        // contact/enable
        auto contact_enable      = m_config.create<IndexT>("contact/enable");
        view(*contact_enable)[0] = b2i(contact["enable"].get<bool>());

        // contact/friction/enable
        auto contact_friction_enable = m_config.create<IndexT>("contact/friction/enable");
        view(*contact_friction_enable)[0] =
            b2i(contact["friction"]["enable"].get<bool>());

        // contact/constitution
        auto contact_constitution = m_config.create<std::string>("contact/constitution");
        view(*contact_constitution)[0] = contact["constitution"].get<std::string>();

        // contact/d_hat
        auto contact_d_hat      = m_config.create<Float>("contact/d_hat");
        view(*contact_d_hat)[0] = contact["d_hat"].get<Float>();

        // contact/eps_velocity
        auto contact_eps_velocity = m_config.create<Float>("contact/eps_velocity");
        view(*contact_eps_velocity)[0] = contact["eps_velocity"].get<Float>();
    }

    // collision_detection
    {
        auto& collision_detection = config["collision_detection"];
        // collision_detection/method
        auto collision_detection_method =
            m_config.create<std::string>("collision_detection/method");
        view(*collision_detection_method)[0] =
            collision_detection["method"].get<std::string>();
    }

    // sanity_check
    {
        auto& sanity_check = config["sanity_check"];
        // sanity_check/enable
        auto sanity_check_enable = m_config.create<IndexT>("sanity_check/enable");
        view(*sanity_check_enable)[0] = b2i(sanity_check["enable"].get<bool>());

        // sanity_check/mode
        auto sanity_check_mode = m_config.create<std::string>("sanity_check/mode");
        view(*sanity_check_mode)[0] = sanity_check["mode"].get<std::string>();
    }

    // recovery
    {
        // now just empty
    }

    // diff_sim
    {
        auto& diff_sim = config["diff_sim"];
        // diff_sim/enable
        auto diff_sim_enable      = m_config.create<IndexT>("diff_sim/enable");
        view(*diff_sim_enable)[0] = b2i(diff_sim["enable"].get<bool>());
    }

    // extras
    {
        auto& extras = config["extras"];
        // extras/debug/dump_surface
        auto extras_debug_dump_surface =
            m_config.create<IndexT>("extras/debug/dump_surface");
        view(*extras_debug_dump_surface)[0] =
            b2i(extras["debug"]["dump_surface"].get<bool>());

        // extras/strict_mode/enable
        auto extras_strict_mode_enable =
            m_config.create<IndexT>("extras/strict_mode/enable");
        view(*extras_strict_mode_enable)[0] =
            b2i(extras["strict_mode"]["enable"].get<bool>());
    }
}

Scene::~Scene() {}
}  // namespace uipc::core::internal