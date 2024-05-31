#pragma once
#include <uipc/world/contact_tabular.h>
#include <uipc/world/constitution_tabular.h>
#include <uipc/world/object.h>
#include <uipc/world/object_slot.h>
#include <uipc/world/object_collection.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::world
{
class Scene
{
    friend class backend::SceneVisitor;
    friend class World;

  public:
    class Objects
    {
        friend class Scene;

      public:
        P<ObjectSlot> create(std::string_view name = "") &&;
        P<ObjectSlot> create(Object&& object) &&;
        P<ObjectSlot> find(IndexT id) && noexcept;
        void          destroy(IndexT id) &&;

      private:
        Objects(Scene& scene) noexcept;
        Scene& m_scene;
    };

    class CObjects
    {
        friend class Scene;

      public:
        P<const ObjectSlot> find(IndexT id) && noexcept;

      private:
        CObjects(const Scene& scene) noexcept;
        const Scene& m_scene;
    };

    Scene() = default;

    ContactTabular&       contact_tabular() noexcept;
    const ContactTabular& contact_tabular() const noexcept;

    ConstitutionTabular&       constitution_tabular() noexcept;
    const ConstitutionTabular& constitution_tabular() const noexcept;

    Objects objects();

  private:
    ContactTabular                              m_contact_tabular;
    ConstitutionTabular                         m_constitution_tabular;
    ObjectCollection                            m_objects;
    bool                                        m_is_running = false;
    const set<IndexT>&                          pending_destroy() noexcept;
    const unordered_map<IndexT, S<ObjectSlot>>& pending_create() noexcept;
    void                                        solve_pending() noexcept;
};
}  // namespace uipc::world
