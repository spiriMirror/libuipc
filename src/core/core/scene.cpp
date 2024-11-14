#include <uipc/core/scene.h>
#include <uipc/common/unit.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>

namespace uipc::core
{
class Scene::Impl
{
  public:
    Impl(Scene& scene, const Json& config) noexcept
        : scene(scene)
        , animator(scene)
    {
        info = config;
    }

    void init(backend::WorldVisitor& world)
    {
        this->world = &world.world();

        backend::SceneVisitor visitor{scene};

        dt = info["dt"].get<Float>();

        spdlog::info("Scene Init: Collecting Constitution UIDs ...");
        constitution_tabular.init(visitor);

        if(info["diff_sim"]["enable"].get<bool>())
        {
            spdlog::info("Scene Init: Initializing Differentiable Simulation Parameters ...");
            diff_sim.init(visitor);
        }

        diff_sim.init(visitor);

        started = true;
    }

    void begin_pending() noexcept
    {
        // Do nothing
    }

    void solve_pending() noexcept
    {
        geometries.solve_pending();
        rest_geometries.solve_pending();
    }

    Json                info;
    ContactTabular      contact_tabular;
    ConstitutionTabular constitution_tabular;
    ObjectCollection    objects;
    Animator            animator;
    DiffSim             diff_sim;

    geometry::GeometryCollection geometries;
    geometry::GeometryCollection rest_geometries;

    bool   started = false;
    Scene& scene;
    World* world = nullptr;
    Float  dt    = 0.0;
};


// ----------------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------------
Json Scene::default_config() noexcept
{
    Json config;
    config["dt"]      = 0.01;
    config["gravity"] = Vector3{0.0, -9.8, 0.0};


    config["cfl"]["enable"] = false;

    auto& newton = config["newton"];
    {
        newton["use_adaptive_tol"] = false;
        newton["velocity_tol"]     = 0.05_m / 1.0_s;
        newton["max_iter"]         = 1024;
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
        contact["eps_velocity"]       = 0.01;
    }

    config["sanity_check"]["enable"] = true;

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
    }

    return config;
}

Scene::Scene(const Json& config)
    : m_impl(uipc::make_unique<Impl>(*this, config))
{
}

Scene::~Scene() {}

ContactTabular& Scene::contact_tabular() noexcept
{
    return m_impl->contact_tabular;
}

const ContactTabular& Scene::contact_tabular() const noexcept
{
    return m_impl->contact_tabular;
}

ConstitutionTabular& Scene::constitution_tabular() noexcept
{
    return m_impl->constitution_tabular;
}
const ConstitutionTabular& Scene::constitution_tabular() const noexcept
{
    return m_impl->constitution_tabular;
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
    return m_impl->info;
}

Animator& Scene::animator()
{
    return m_impl->animator;
}

const Animator& Scene::animator() const
{
    return m_impl->animator;
}

DiffSim& Scene::diff_sim()
{
    UIPC_ASSERT(false, "Not fully implemented yet in this version.");
    return m_impl->diff_sim;
}

const DiffSim& Scene::diff_sim() const
{
    UIPC_ASSERT(false, "Not fully implemented yet in this version.");
    return m_impl->diff_sim;
}

void Scene::init(backend::WorldVisitor& world)
{
    m_impl->init(world);
}

void Scene::begin_pending() noexcept
{
    m_impl->begin_pending();
}

void Scene::solve_pending() noexcept
{
    m_impl->solve_pending();
}

geometry::GeometryCollection& Scene::geometry_collection() noexcept
{
    return m_impl->geometries;
}

geometry::GeometryCollection& Scene::rest_geometry_collection() noexcept
{
    return m_impl->rest_geometries;
}

World& Scene::world() noexcept
{
    return *m_impl->world;
}

Float Scene::dt() const noexcept
{
    return m_impl->dt;
}

bool Scene::is_started() const noexcept
{
    return m_impl->started;
}

// ----------------------------------------------------------------------------
// Objects
// ----------------------------------------------------------------------------
S<Object> Scene::Objects::create(std::string_view name) &&
{
    auto id = m_scene.m_impl->objects.m_next_id;
    return m_scene.m_impl->objects.emplace(Object{m_scene, id, name});
}

S<Object> Scene::Objects::find(IndexT id) && noexcept
{
    return m_scene.m_impl->objects.find(id);
}

void Scene::Objects::destroy(IndexT id) &&
{
    auto obj = m_scene.m_impl->objects.find(id);
    if(!obj)
    {
        UIPC_WARN_WITH_LOCATION("Trying to destroy non-existing object ({}), ignored.", id);
        return;
    }

    auto geo_ids = obj->geometries().ids();

    for(auto geo_id : geo_ids)
    {
        if(!m_scene.m_impl->started)
        {
            m_scene.m_impl->geometries.destroy(geo_id);
            m_scene.m_impl->rest_geometries.destroy(geo_id);
        }
        else
        {
            m_scene.m_impl->geometries.pending_destroy(geo_id);
            m_scene.m_impl->rest_geometries.pending_destroy(geo_id);
        }
    }
    m_scene.m_impl->objects.destroy(id);
}

SizeT Scene::Objects::size() const noexcept
{
    return m_scene.m_impl->objects.size();
}

S<const Object> Scene::CObjects::find(IndexT id) && noexcept
{
    return m_scene.m_impl->objects.find(id);
}

SizeT Scene::CObjects::size() const noexcept
{
    return m_scene.m_impl->objects.size();
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

ObjectGeometrySlots<geometry::Geometry> core::Scene::Geometries::find(IndexT id) && noexcept
{
    return {m_scene.m_impl->geometries.find(id), m_scene.m_impl->rest_geometries.find(id)};
}

ObjectGeometrySlots<const geometry::Geometry> Scene::CGeometries::find(IndexT id) && noexcept
{
    return {m_scene.m_impl->geometries.find(id), m_scene.m_impl->rest_geometries.find(id)};
}
}  // namespace uipc::core
