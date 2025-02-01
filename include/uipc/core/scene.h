#pragma once
#include <uipc/core/contact_tabular.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/core/object.h>
#include <uipc/core/object_collection.h>
#include <uipc/core/animator.h>
#include <uipc/core/diff_sim.h>
#include <uipc/core/sanity_checker.h>

namespace uipc::backend
{
class SceneVisitor;
class WorldVisitor;
}  // namespace uipc::backend

namespace uipc::core
{
class UIPC_CORE_API Scene final
{
    friend class backend::SceneVisitor;
    friend class World;
    friend class Object;
    friend class SanityChecker;
    friend class Animation;

  public:
    Scene(const Json& config = default_config());
    ~Scene();

    static Json default_config() noexcept;

    class UIPC_CORE_API Objects
    {
        friend class Scene;

      public:
        S<Object> create(std::string_view name = "") &&;
        S<Object> find(IndexT id) && noexcept;
        void      destroy(IndexT id) &&;
        SizeT     size() const noexcept;
        SizeT     created_count() const noexcept;

      private:
        Objects(Scene& scene) noexcept;
        Scene& m_scene;
    };

    class UIPC_CORE_API CObjects
    {
        friend class Scene;

      public:
        S<const Object> find(IndexT id) && noexcept;
        SizeT           size() const noexcept;
        SizeT           created_count() const noexcept;

      private:
        CObjects(const Scene& scene) noexcept;
        const Scene& m_scene;
    };

    class UIPC_CORE_API Geometries
    {
        friend class Scene;

      public:
        ObjectGeometrySlots<geometry::Geometry> find(IndexT id) && noexcept;

      private:
        Geometries(Scene& scene) noexcept;
        Scene& m_scene;
    };

    class UIPC_CORE_API CGeometries
    {
        friend class Scene;

      public:
        ObjectGeometrySlots<const geometry::Geometry> find(IndexT id) && noexcept;

      private:
        CGeometries(const Scene& scene) noexcept;
        const Scene& m_scene;
    };

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

  private:
    class Impl;
    U<Impl> m_impl;
    friend class SanityChecker;

    void init(backend::WorldVisitor& world);  // only be called by World.

    void begin_pending() noexcept;
    void solve_pending() noexcept;

    geometry::GeometryCollection& geometry_collection() noexcept;
    geometry::GeometryCollection& rest_geometry_collection() noexcept;

    World& world() noexcept;
    Float  dt() const noexcept;
    bool   is_started() const noexcept;
    bool   is_pending() const noexcept;
};
}  // namespace uipc::core
