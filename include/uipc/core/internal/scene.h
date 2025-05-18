#pragma once
#include <uipc/core/contact_tabular.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/core/object.h>
#include <uipc/core/object_collection.h>
#include <uipc/core/animator.h>
#include <uipc/core/diff_sim.h>
#include <uipc/core/sanity_checker.h>
#include <uipc/core/internal/world.h>

namespace uipc::core
{
class SceneSnapshotCommit;
}

namespace uipc::core::internal
{
class World;

class UIPC_CORE_API Scene : public std::enable_shared_from_this<Scene>
{
    friend class core::Scene;

  public:
    Scene(const Json& config) noexcept;

    ~Scene();

    void init(internal::World& world) noexcept;

    void begin_pending() noexcept;

    void solve_pending() noexcept;

    void update_from(const SceneSnapshotCommit& commit);

    auto& config() const noexcept { return m_config; }
    auto& config() noexcept { return m_config; }
    Float dt() const noexcept { return m_config["dt"].get<Float>(); }
    bool  is_started() const noexcept { return m_started; }
    bool  is_pending() const noexcept { return m_pending; }
    auto& contact_tabular() const noexcept { return m_contact_tabular; }
    auto& contact_tabular() noexcept { return m_contact_tabular; }
    auto& constitution_tabular() const noexcept
    {
        return m_constitution_tabular;
    }
    auto& constitution_tabular() noexcept { return m_constitution_tabular; }
    auto& animator() const noexcept { return m_animator; }
    auto& animator() noexcept { return m_animator; }
    auto& diff_sim() const noexcept { return m_diff_sim; }
    auto& diff_sim() noexcept { return m_diff_sim; }
    auto  world() noexcept { return m_world; }
    auto& objects() const noexcept { return m_objects; }
    auto& objects() noexcept { return m_objects; }
    auto& geometries() const noexcept { return m_geometries; }
    auto& geometries() noexcept { return m_geometries; }
    auto& rest_geometries() const noexcept { return m_rest_geometries; }
    auto& rest_geometries() noexcept { return m_rest_geometries; }
    auto& sanity_checker() const noexcept { return m_sanity_checker; }
    auto& sanity_checker() noexcept { return m_sanity_checker; }

  private:
    Json                m_config;
    ContactTabular      m_contact_tabular;
    ConstitutionTabular m_constitution_tabular;
    ObjectCollection    m_objects;
    Animator            m_animator;
    DiffSim             m_diff_sim;

    geometry::GeometryCollection m_geometries;
    geometry::GeometryCollection m_rest_geometries;
    SanityChecker                m_sanity_checker;

    bool             m_started = false;
    bool             m_pending = false;
    internal::World* m_world   = nullptr;
    Float            m_dt      = 0.0;
};
}  // namespace uipc::core::internal