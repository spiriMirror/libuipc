#pragma once
#include <uipc/core/contact_tabular.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/core/object.h>
#include <uipc/core/object_collection.h>
#include <uipc/core/animator.h>
#include <uipc/core/diff_sim.h>
#include <uipc/core/sanity_checker.h>
namespace uipc::core::internal
{
class Scene;
class World;
}  // namespace uipc::core::internal

namespace uipc::backend
{
class SceneVisitor;
class WorldVisitor;
}  // namespace uipc::backend

namespace uipc::sanity_check
{
class SanityChecker;
}

namespace uipc::core
{
class SceneSnapshotCommit;
class SceneSnapshot;

class UIPC_CORE_API Scene final
{
    friend class backend::SceneVisitor;
    friend class World;
    friend class Object;
    friend class sanity_check::SanityChecker;
    friend class Animation;
    friend class SceneFactory;
    friend class SceneSnapshot;
    friend struct fmt::formatter<Scene>;

  public:
    explicit Scene(const Json& config = default_config());

    Scene(const Scene&) = delete;
    Scene(Scene&&)      = default;

    ~Scene();

    static Json default_config() noexcept;

    class UIPC_CORE_API Objects
    {
        friend class Scene;

      public:
        S<Object>         create(std::string_view name = "") &&;
        S<Object>         find(IndexT id) && noexcept;
        vector<S<Object>> find(std::string_view name) && noexcept;
        void              destroy(IndexT id) &&;
        SizeT             size() const noexcept;
        SizeT             created_count() const noexcept;

      private:
        Objects(internal::Scene& scene) noexcept;
        internal::Scene& m_scene;
    };

    class UIPC_CORE_API CObjects
    {
        friend class Scene;

      public:
        S<const Object>         find(IndexT id) && noexcept;
        vector<S<const Object>> find(std::string_view name) && noexcept;
        SizeT                   size() const noexcept;
        SizeT                   created_count() const noexcept;

      private:
        CObjects(const internal::Scene& scene) noexcept;
        const internal::Scene& m_scene;
    };

    class UIPC_CORE_API Geometries
    {
        friend class Scene;

      public:
        ObjectGeometrySlots<geometry::Geometry> find(IndexT id) && noexcept;

      private:
        Geometries(internal::Scene& scene) noexcept;
        internal::Scene& m_scene;
    };

    class UIPC_CORE_API CGeometries
    {
        friend class Scene;

      public:
        ObjectGeometrySlots<const geometry::Geometry> find(IndexT id) && noexcept;

      private:
        CGeometries(const internal::Scene& scene) noexcept;
        const internal::Scene& m_scene;
    };

    const Json& config() const noexcept;

    ContactTabular&       contact_tabular() noexcept;
    const ContactTabular& contact_tabular() const noexcept;

    ConstitutionTabular&       constitution_tabular() noexcept;
    const ConstitutionTabular& constitution_tabular() const noexcept;

    Objects  objects() noexcept;
    CObjects objects() const noexcept;

    Geometries  geometries() noexcept;
    CGeometries geometries() const noexcept;

    const Json& info() const noexcept;

    Animator&       animator();
    const Animator& animator() const;

    DiffSim&       diff_sim();
    const DiffSim& diff_sim() const;

    SanityChecker&       sanity_checker();
    const SanityChecker& sanity_checker() const;

    void update_from(const SceneSnapshotCommit& snapshot);

  private:
    // Allow create a core::Scene from a core::internal::Scene
    Scene(S<internal::Scene> scene) noexcept;
    S<core::internal::Scene> m_internal;
};
}  // namespace uipc::core

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::core::Scene> : formatter<string_view>
{
    appender format(const uipc::core::Scene& c, format_context& ctx) const;
};
}  // namespace fmt