#include <uipc/core/scene.h>
#include <uipc/common/unit.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/internal/scene.h>
#include <uipc/core/scene_snapshot.h>

namespace uipc::core
{
// ----------------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------------
Json Scene::default_config() noexcept
{
    Json config;
    config["dt"]      = 0.01;
    config["gravity"] = Vector3{0.0, -9.8, 0.0};


    config["cfl"]["enable"] = false;

    auto& integrator = config["integrator"];
    {
        integrator["type"] = "bdf1"; // bdf1
    }

    auto& newton = config["newton"];
    {
        newton["max_iter"] = 1024;

        newton["use_adaptive_tol"] = false;

        // convergence tolerance
        // 1) max dx <= velocity_tol * dt
        newton["velocity_tol"] = 0.05_m / 1.0_s;
        // 2) ccd_toi >= ccd_tol
        newton["ccd_tol"] = 1.0;
        // 3) max dF <=  dF <= transform_tol * dt
        newton["transrate_tol"] = 0.1 / 1.0_s;  // 10%/s change in transform
    }

    auto& linear_system = config["linear_system"];
    {
        linear_system["tol_rate"] = 1e-3;
        linear_system["solver"]   = "linear_pcg";
    }

    auto& line_search = config["line_search"];
    {
        line_search["max_iter"]      = 8;
        line_search["report_energy"] = false;
    }

    auto& contact = config["contact"];
    {
        contact["enable"]             = true;
        contact["friction"]["enable"] = true;
        contact["constitution"]       = "ipc";
        contact["d_hat"]              = 0.01;
        contact["eps_velocity"]       = 0.01_m / 1.0_s;
    }

    auto& collision_detection = config["collision_detection"];
    {
        collision_detection["method"] = "linear_bvh";
    }

    auto& sanity_check = config["sanity_check"];
    {
        sanity_check["enable"] = true;

        // normal: automatically export mesh to workspace
        // quiet: do not export mesh
        sanity_check["mode"] = "normal";
    }

    auto& recovery = config["recovery"] = Json::object();
    {
        // now just empty
    }

    auto& diff_sim = config["diff_sim"] = Json::object();
    {
        diff_sim["enable"] = false;
    }

    // something that is unofficial
    auto& extras = config["extras"] = Json::object();
    {
        extras["debug"]["dump_surface"] = false;
        extras["strict_mode"]["enable"] = false;
    }

    return config;
}

Scene::Scene(const Json& config)
    : m_internal(uipc::make_shared<internal::Scene>(config))
{
}

Scene::Scene(S<internal::Scene> scene) noexcept
    : m_internal(std::move(scene))
{
}

const Json& Scene::config() const noexcept
{
    return m_internal->config();
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

const Json& Scene::info() const noexcept
{
    return m_internal->config();
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
    m_internal->config()["diff_sim"]["enable"] = true;
    return m_internal->diff_sim();
}

const DiffSim& Scene::diff_sim() const
{
    return m_internal->diff_sim();
}

SanityChecker& Scene::sanity_checker()
{
    return m_internal->sanity_checker();
}

const SanityChecker& Scene::sanity_checker() const
{
    return m_internal->sanity_checker();
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
    fmt::format_to(ctx.out(), "{}", c.m_internal->objects());
    fmt::format_to(ctx.out(), "\n{}", c.m_internal->animator());
    return ctx.out();
}
}  // namespace fmt