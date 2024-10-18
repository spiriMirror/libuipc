#pragma once
#include <uipc/core/contact_tabular.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/core/object.h>
#include <uipc/core/object_collection.h>
#include <uipc/core/animator.h>
#include <uipc/core/diff_sim.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::core
{
class UIPC_CORE_API Scene
{
    friend class backend::SceneVisitor;
    friend class World;
    friend class Object;

  public:
    Scene(const Json& config = default_config());

    static Json default_config() noexcept;

    class UIPC_CORE_API Objects
    {
        friend class Scene;

      public:
        S<Object> create(std::string_view name = "") &&;
        S<Object> find(IndexT id) && noexcept;
        void      destroy(IndexT id) &&;
        SizeT     size() const noexcept;

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

  private:
    friend class SanityChecker;
    friend class Animation;

    class Impl
    {
      public:
        Impl(Scene& s) noexcept;
        Float               dt = 0.0;
        Json                info;
        ContactTabular      contact_tabular;
        ConstitutionTabular constitution_tabular;
        ObjectCollection    objects;
        Animator            animator;
        DiffSim             diff_sim;

        geometry::GeometryCollection geometries;
        geometry::GeometryCollection rest_geometries;

        bool   started = false;
        World* world   = nullptr;
    };

    Impl m_impl;

    void solve_pending() noexcept;
};
}  // namespace uipc::core
